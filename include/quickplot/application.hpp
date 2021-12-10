#include <mutex>
#include <string>
#include <utility>
#include <memory>
#include <algorithm>
#include <map>
#include <list>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <imgui_internal.h>
#include <boost/algorithm/string/join.hpp>
#include <rosidl_typesupport_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>
#include "quickplot/config.hpp"
#include "quickplot/node.hpp"
#include "quickplot/plot_view.hpp"
#include <rcpputils/asserts.hpp>

namespace fs = std::filesystem;

namespace quickplot
{

constexpr const char * TOPIC_LIST_WINDOW_ID = "TopicList";
constexpr float HOVER_TOOLTIP_DEBOUNCE = 0.4;

struct MemberPayload
{
  const char * topic_name;
  size_t member_index;
};

inline std::string source_id(const DataSource & source)
{
  std::stringstream ss;
  ss << source.resolved_topic_name << "/";
  for (size_t i = 0; i < source.config.member_path.size(); i++) {
    ss << source.config.member_path[i];
    if (i + 1 < source.config.member_path.size()) {
      ss << ".";
    }
  }
  return ss.str();
}

class Application
{
private:
  std::shared_ptr<QuickPlotNode> node_;
  rclcpp::Event::SharedPtr graph_event_;
  rclcpp::JumpHandler::SharedPtr jump_handler_;

  // maps already-resolved full topic names to a single ROS type, if there are only publishers
  // of one type
  std::map<std::string, std::string> available_topics_to_types_;
  std::unordered_map<std::string,
    std::shared_ptr<MessageIntrospection>> message_type_to_introspection_;

  // set of message types for which no typesupport is installed
  std::unordered_set<std::string> message_type_unavailable_;

  double history_length_;
  // mutex for plot metadata
  std::mutex plot_mutex_;
  // plot metadata
  std::vector<Plot> plots_;

public:
  explicit Application(std::shared_ptr<QuickPlotNode> _node)
  : node_(_node), history_length_(1.0), plots_()
  {
    graph_event_ = node_->get_graph_event();
    graph_event_->set(); // set manually to trigger initial topics query

    jump_handler_ = node_->get_clock()->create_jump_callback(
      [] {}, std::bind(&Application::on_time_jump, this, _1), rcl_jump_threshold_t {
        .on_clock_change = true,
        .min_forward = {
          .nanoseconds = RCUTILS_S_TO_NS(10),
        },
        .min_backward = {
          .nanoseconds = -1,
        },
      });
  }

  void on_time_jump(const rcl_time_jump_t & time_jump)
  {
    if (time_jump.clock_change == RCL_ROS_TIME_ACTIVATED ||
      time_jump.clock_change == RCL_ROS_TIME_DEACTIVATED)
    {
      std::cerr << "clock changed";
    } else if (time_jump.delta.nanoseconds != 0) {
      // check should not be necessary
      // fix merged with https://github.com/ros2/rcl/pull/948 and should be in ROS Humble
      std::cerr << "time jump by a delta of " << time_jump.delta.nanoseconds << "ns";
    }
    std::cerr << ", clearing all data" << std::endl;
    clear_data();
  }

  void apply_config(ApplicationConfig config)
  {
    {
      std::unique_lock<std::mutex> lock(plot_mutex_);
      plots_.clear();
      std::transform(
        config.plots.begin(), config.plots.end(), std::back_inserter(plots_),
        [](const auto & plot_config) {
          Plot p;
          for (const auto & s : plot_config.sources) {
            auto & new_s = p.sources.emplace_back();
            new_s.config = s;
            new_s.state = DataSourceState::Uninitialized;
          }
          p.axes = plot_config.axes;
          return p;
        });
    }
    history_length_ = config.history_length;
    initialize_pending_sources();
  }

  ApplicationConfig get_config() const
  {
    ApplicationConfig config;
    config.history_length = history_length_;
    std::transform(
      plots_.begin(), plots_.end(), std::back_inserter(config.plots), [](const auto & plot) {
        PlotConfig p;
        for (const auto & s : plot.sources) {
          p.sources.insert(s.config);
        }
        p.axes = plot.axes;
        return p;
      });
    return config;
  }

  void load_introspection_or_insert_missing(std::string type)
  {
    if (message_type_to_introspection_.find(type) == message_type_to_introspection_.end()) {
      if (message_type_unavailable_.find(type) == message_type_unavailable_.end()) {
        try {
          message_type_to_introspection_.emplace(
            type,
            std::make_shared<MessageIntrospection>(type));
        } catch (const introspection_error &) {
          // TODO(ZeilingerM) store specific error to display on GUI
          message_type_unavailable_.insert(type);
        }
      }
    }
  }

  void initialize_pending_sources()
  {
    std::unique_lock<std::mutex> lock(plot_mutex_);
    for (auto & plot : plots_) {
      for (auto & source : plot.sources) {
        if (source.state == DataSourceState::Uninitialized) {
          auto resolved_topic = node_->get_node_topics_interface()->resolve_topic_name(
            source.config.topic_name);
          source.resolved_topic_name = resolved_topic;
          source.id = source_id(source);
          auto type_it = available_topics_to_types_.find(resolved_topic);
          if (type_it != available_topics_to_types_.end()) {
            // if type is known, initialize and set ready state
            const auto & itsp_it = message_type_to_introspection_.find(type_it->second);
            if (itsp_it != message_type_to_introspection_.end()) {
              MessageMember member;
              auto member_info_opt = itsp_it->second->get_member_info(source.config.member_path);
              if (member_info_opt.has_value()) {
                member.path = source.config.member_path;
                member.info = member_info_opt.value();
                node_->add_topic_field(resolved_topic, itsp_it->second, member);
                source.state = DataSourceState::Ok;
              } else {
                source.state = DataSourceState::InvalidMember;
              }
            }
            // if type is not known, we leave the source at initialized, and the GUI should show
            // the state as unavailable
          }
        }
      }
    }
  }

  bool TopicEntry(const std::string & topic, const std::string & type)
  {
    ImGuiTreeNodeFlags tree_node_flags = ImGuiTreeNodeFlags_None;

    // for any topic type, there must be either
    // * a type introspection library loaded
    // * the type confirmed to be unavailable
    bool type_available = message_type_unavailable_.find(type) == message_type_unavailable_.end();
    if (!type_available) {
      ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyle().Colors[ImGuiCol_TextDisabled]);
      tree_node_flags |= ImGuiTreeNodeFlags_Leaf;
    }

    // the type-to-introspection map is assumed to contain the type key, since unavailable key is
    // handled at this point
    if (ImGui::TreeNodeEx(topic.c_str(), tree_node_flags)) {
      if (type_available) {
        if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_SourceAllowNullID)) {
          ImGui::SetDragDropPayload("topic_name", topic.c_str(), topic.size());
          ImPlot::ItemIcon(ImGui::GetColorU32(ImGuiCol_Text));
          ImGui::SameLine();
          ImGui::Text("Dragging %s", topic.c_str());
          ImGui::EndDragDropSource();
        }
        auto introspection = message_type_to_introspection_.at(type);
        size_t i {0};
        for (const auto & member : introspection->members()) {
          std::string member_formatted = boost::algorithm::join(member.path, ".");
          ImGui::Selectable(member_formatted.c_str(), false);
          if (ImGui::IsItemHovered()) {
            ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);

            if (GImGui->HoveredIdTimer > 0.5) {
              ImGui::BeginTooltip();
              ImGui::Text("add %s to plot 0", member_formatted.c_str());
              ImGui::Text("or drag and drop on plot of choice");
              ImGui::EndTooltip();
            }
          }
          MemberPayload payload {
            .topic_name = topic.c_str(),
            .member_index = i,
          };
          if (ImGui::IsItemActivated()) {
            activate_topic_field(payload);
          }
          if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_SourceAllowNullID)) {
            ImGui::SetDragDropPayload("topic_member", &payload, sizeof(payload));
            ImGui::EndDragDropSource();
          }
          ++i;
        }
      } else { /* not type_available */
        ImGui::PopStyleColor();

        // color was eye-dropped from Rviz display warnings
        // TODO(ZeilingerM) single hardcoded color is not robust against theme changes
        static ImVec4 WARNING_COLOR = static_cast<ImVec4>(ImColor::HSV(0.1083f, 0.968f, 0.867f));
        ImGui::PushStyleColor(ImGuiCol_Text, WARNING_COLOR);
        ImGui::TextWrapped("message type '%s' is not available", type.c_str());
        ImGui::PopStyleColor();
      }
      return true;
    }
    if (!type_available) {
      ImGui::PopStyleColor();
    }
    return false;
  }

  void EndTopicEntry()
  {
    ImGui::TreePop();
  }

  void set_source_states(const std::string & resolved_topic_name, DataSourceState state)
  {
    for (auto & plot : plots_) {
      for (auto & source : plot.sources) {
        if (source.resolved_topic_name == resolved_topic_name) {
          source.state = state;
        }
      }
    }
  }

  void update_topics()
  {
    if (graph_event_->check_and_clear()) {
      auto topics_and_types = node_->get_topic_names_and_types();
      for (const auto & [topic, types] : topics_and_types) {
        if (types.size() != 1) {
          std::cerr << "topic " << topic << " has multiple types, quickplot will ignore it" <<
            std::endl;
          continue;
        }
        auto new_type = types[0];
        auto [entry, inserted] = available_topics_to_types_.try_emplace(topic, new_type);
        if (!inserted) {
          // the topic was already in the map, so we possibly have active listeners
          // parsing a message type that may have changed
          if (entry->second != new_type) {
            std::invalid_argument("type of topic " + topic + " changed");
          }
        } else {
          // for new topics, load their introspection support
          load_introspection_or_insert_missing(new_type);
        }
      }
      initialize_pending_sources();
    }
  }

  void detect_clock_issues(const PlotViewOptions & plot_opts)
  {
    std::unique_lock<std::mutex> lock(node_->topic_mutex);
    auto start_sec = plot_opts.t_start.seconds();
    auto end_sec = plot_opts.t_end.seconds();

    for (auto & [topic, subscription] : node_->topics_to_subscriptions) {
      for (auto & [_, buffer] : subscription.buffers_) {
        //
        // Check received data for clock mismatch issues.
        //
        // 1) The plotting tool uses the system clock, but data is published in ros clock.
        //    In this case the data buffer would be cleared, because the plot start timestamp will be much larger than the last data timestamp
        bool clock_issue_likely = false;
        bool clock_issue_disproven = false;
        bool had_data = !buffer.empty();
        buffer.clear_data_up_to(plot_opts.t_start);
        if (had_data) {
          if (buffer.empty()) {
            clock_issue_likely = buffer.empty();
          } else {
            clock_issue_disproven = true;
          }
        }

        // 2) The plotting tool uses the ROS clock, but data is published with system time stamps
        //    In this case all data would be outside the view window, with much larger timestamps, and slowly accumulate
        if (!clock_issue_likely) {
          auto data = buffer.data();
          clock_issue_likely = std::all_of(
            data.begin(), data.end(),
            [start_sec, end_sec](const ImPlotPoint & item) {
              return item.x < start_sec || item.x > end_sec;
            });
        }
        if (clock_issue_likely) {
          set_source_states(topic, DataSourceState::TimeStampOutOfRange);
        } else if (clock_issue_disproven) {
          set_source_states(topic, DataSourceState::Ok);
        }
      }
    }
  }

  void update()
  {
    update_topics();
    TopicList();

    auto t = node_->now();
    auto history_dur = rclcpp::Duration::from_seconds(history_length_);

    auto plot_opts = PlotViewOptions {
      .use_sim_time = node_->get_parameter("use_sim_time").as_bool(),
      .t_start = t - history_dur,
      .t_end = t,
    };
    detect_clock_issues(plot_opts);
    PlotDock(plot_opts);
  }

  void TopicList()
  {
    ImGuiWindowFlags list_window_flags = ImGuiWindowFlags_None;
    if (ImGui::Begin(TOPIC_LIST_WINDOW_ID, nullptr, list_window_flags)) {
      if (!node_->topics_to_subscriptions.empty()) {
        if (ImGui::CollapsingHeader("active topics", ImGuiTreeNodeFlags_DefaultOpen)) {
          for (const auto & [topic, subscription] : node_->topics_to_subscriptions) {
            rcpputils::assert_true(
              available_topics_to_types_.find(
                topic) != available_topics_to_types_.end(),
              "topics can only become active when their type is known");
            auto type = available_topics_to_types_[topic];
            if (TopicEntry(topic, type)) {
              auto stats = subscription.receive_period_stats();
              if (stats.standard_deviation < (stats.average / 10.0) && stats.average < 1.0 &&
                stats.average > 0.001)
              {
                ImGui::Text("%.1f hz", 1.0 / stats.average);
              }
              EndTopicEntry();
            }
          }
        }
      }
      if (ImGui::CollapsingHeader("available topics", ImGuiTreeNodeFlags_DefaultOpen)) {
        // according to https://design.ros2.org/articles/topic_and_service_names.html, max topic
        // name length is 256
        const size_t filter_text_length = 256 + 1;
        static char filter_text[filter_text_length];
        ImGui::PushItemWidth(-1);
        ImGui::InputTextWithHint("", "filter topic or type", filter_text, filter_text_length);
        ImGui::PopItemWidth();

        std::vector<std::pair<std::string, std::string>> shown_available_topics;
        size_t total_available = 0;
        for (const auto & [topic, type] : available_topics_to_types_) {
          if (node_->topics_to_subscriptions.find(topic) != node_->topics_to_subscriptions.end()) {
            continue;
          }
          ++total_available;
          if (!strstr(topic.c_str(), filter_text)) {
            continue;
          }
          shown_available_topics.push_back({topic, type});
        }

        if (filter_text[0] != '\0') {
          // if filter is passed, show amount of filtered topics
          ImGui::PushItemWidth(-1);
          ImGui::Text("Showing %lu of %lu topics", shown_available_topics.size(), total_available);
          ImGui::PopItemWidth();
        }
        for (const auto & [topic, type] : shown_available_topics) {
          if (TopicEntry(topic, type)) {
            EndTopicEntry();
          }
        }
      }
    }
    ImGui::End();
  }

  void PlotDock(const PlotViewOptions & plot_opts)
  {
    std::unique_lock<std::mutex> lock(node_->topic_mutex);

    auto plot_it = plots_.begin();
    for (size_t i = 0; plot_it != plots_.end(); i++) {
      auto id = "plot" + std::to_string(i);
      bool plot_window_enabled = true;

      // for plot windows, the padding is redundant with dock borders
      ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0, 0.0));
      if (ImGui::Begin(id.c_str(), &plot_window_enabled)) {
        ImGui::PopStyleVar();

        if (PlotView(id.c_str(), *plot_it, plot_opts)) {
          for (ImPlotYAxis a = 0; a < static_cast<ImPlotYAxis>(plot_it->axes.size()); a++) {
            ImPlot::SetPlotYAxis(a);
            for (auto source_it = plot_it->sources.begin(); source_it != plot_it->sources.end(); ) {
              auto & source = *source_it;
              if (source.config.axis != a) {
                // source does not occur in this plot axis
                continue;
              }

              if (source.state == DataSourceState::Ok) {
                auto it = node_->topics_to_subscriptions.find(source.resolved_topic_name);
                rcpputils::require_true(it != node_->topics_to_subscriptions.end());
                PlotSource(source, it->second.get_buffer(source.config.member_path).data());
              } else {
                PlotSourceError(source, plot_opts);
              }

              if (PlotSourcePopup(source)) {
                source_it = plot_it->sources.erase(source_it);
              } else {
                ++source_it;
              }
            }
            auto & axis_config = plot_it->axes[a];
            // store the current plot limits, in case they were changed via user input
            auto limits = ImPlot::GetPlotLimits(a);
            axis_config.y_min = limits.Y.Min;
            axis_config.y_max = limits.Y.Max;
          }

          if (ImPlot::BeginDragDropTarget()) {
            if (const ImGuiPayload * payload = ImGui::AcceptDragDropPayload("topic_name")) {
              auto topic_name = std::string(static_cast<char *>(payload->Data));
              MemberPayload member_payload {
                .topic_name = topic_name.c_str(),
                .member_index = 1,
              };
              accept_member_payload(*plot_it, ImPlotYAxis_1, &member_payload);
            }
            if (const ImGuiPayload * payload = ImGui::AcceptDragDropPayload("topic_member")) {
              accept_member_payload(
                *plot_it, ImPlotYAxis_1,
                static_cast<MemberPayload *>(payload->Data));
            }
            ImPlot::EndDragDropTarget();
          }
          for (auto axis : {ImPlotYAxis_1, ImPlotYAxis_2, ImPlotYAxis_3}) {
            if (ImPlot::BeginDragDropTargetY(axis)) {
              if (const ImGuiPayload * payload = ImGui::AcceptDragDropPayload("topic_name")) {
                auto topic_name = std::string(static_cast<char *>(payload->Data));
                MemberPayload member_payload {
                  .topic_name = topic_name.c_str(),
                  .member_index = 1,
                };
                accept_member_payload(*plot_it, axis, &member_payload);
              }
              if (const ImGuiPayload * payload = ImGui::AcceptDragDropPayload("topic_member")) {
                accept_member_payload(
                  *plot_it, axis,
                  static_cast<MemberPayload *>(payload->Data));
              }
              ImPlot::EndDragDropTarget();
            }
          }
          EndPlotView();
        }
        if (!plot_window_enabled) {
          plot_it = plots_.erase(plot_it);
        } else {
          ++plot_it;
        }
      }
      ImGui::End();
    }
  }

  void activate_topic_field(MemberPayload payload)
  {
    if (plots_.empty()) {
      auto & new_plot = plots_.emplace_back();
      new_plot.axes = {AxisConfig{.y_min = -1, .y_max = 1}};
    } else if (plots_[0].axes.empty()) {
      auto& new_axis = plots_[0].axes.emplace_back();
      new_axis.y_min = -1.0;
      new_axis.y_max = 1.0;
    }
    accept_member_payload(plots_[0], ImPlotYAxis_1, &payload);
  }

  void accept_member_payload(Plot & plot, ImPlotYAxis axis, MemberPayload * payload)
  {
    auto message_type = available_topics_to_types_.at(payload->topic_name);
    auto introspection = message_type_to_introspection_.at(message_type);
    auto member_infos = introspection->members().begin();
    std::advance(member_infos, payload->member_index);
    node_->add_topic_field(payload->topic_name, introspection, *member_infos);

    DataSourceConfig source_config;
    source_config.topic_name = payload->topic_name;
    source_config.member_path = member_infos->path;
    source_config.axis = axis;

    auto it = std::find_if(plot.sources.begin(), plot.sources.end(), [&source_config](const auto& src) {
      return src.config == source_config;
    });
    if (it != plot.sources.end()) {
      // ensure the same source config is not added twice
      return;
    }
    auto & source = plot.sources.emplace_back();
    source.resolved_topic_name = payload->topic_name;
    // assume the state is Ok, since the topic was drag-dropped from the available list
    source.state = DataSourceState::Ok;
    source.config = source_config;
    source.id = source_id(source);
  }

  void clear_data()
  {
    for (auto & [_, subscription]: node_->topics_to_subscriptions) {
      subscription.clear_all_data();
    }
  }
};

} // namespace quickplot
