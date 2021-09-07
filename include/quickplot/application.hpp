#include <mutex>
#include <string>
#include <utility>
#include <memory>
#include <deque>
#include <list>
#include <unordered_set>
#include <unordered_map>
#include <filesystem>
#include <imgui_internal.h>
#include <rosidl_typesupport_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>
#include "quickplot/config.hpp"
#include "quickplot/node.hpp"
#include "quickplot/plot_view.hpp"
#include <rcpputils/asserts.hpp>

namespace fs = std::filesystem;

namespace quickplot
{

constexpr float HOVER_TOOLTIP_DEBOUNCE = 0.4;

class Application
{
private:
  fs::path active_config_file_;
  ApplicationConfig config_;

  std::shared_ptr<QuickPlotNode> node_;
  rclcpp::Event::SharedPtr graph_event_;
  rclcpp::JumpHandler::SharedPtr jump_handler_;

  double edited_history_length_;

  std::unordered_map<std::string, std::string> available_topics_to_types_;
  std::unordered_map<std::string,
    std::shared_ptr<MessageIntrospection>> message_type_to_introspection_;

  // set of message types for which no typesupport is installed
  std::unordered_set<std::string> message_type_unavailable_;

  // queue of requested topic data, for which the topic metadata has not been received yet
  std::mutex data_source_queue_mutex_;
  std::deque<DataSourceConfig> uninitialized_data_source_queue_;

  std::unordered_set<std::string> topics_with_time_reference_issues_;

public:
  explicit Application(fs::path config_file, std::shared_ptr<QuickPlotNode> _node)
  : active_config_file_(config_file), node_(_node)
  {
    graph_event_ = node_->get_graph_event();
    graph_event_->set(); // set manually to trigger initial topics query

    jump_handler_ = node_->get_clock()->create_jump_callback(
      [] {}, std::bind(&Application::on_time_jump, this, _1), rcl_jump_threshold_t {
        .on_clock_change = true,
        .min_forward = {
          .nanoseconds = RCUTILS_S_TO_NS(1),
        },
        .min_backward = {
          .nanoseconds = -RCUTILS_S_TO_NS(1),
        },
      });

    try {
      config_ = load_config(active_config_file_);
    } catch (const config_error & e) {
      std::cerr << "Failed to read configuration from '" << config_file.c_str() << "'" << std::endl;
      print_exception_recursive(e, 0, 1);
      std::cout << "Using default configuration" << std::endl;
      config_ = default_config();
    }
    apply_config(config_);
  }

  void on_time_jump(const rcl_time_jump_t & time_jump)
  {
    std::cerr << "time jump occured ";
    if (time_jump.clock_change == RCL_ROS_TIME_ACTIVATED ||
      time_jump.clock_change == RCL_ROS_TIME_DEACTIVATED)
    {
      std::cerr << "due to clock change";
    } else {
      std::cerr << "by a delta of " << time_jump.delta.nanoseconds << "ns";
    }
    std::cerr << ", clearing all data" << std::endl;
    for (auto & [_, subscription]: node_->topics_to_subscriptions) {
      subscription.clear_all_data();
    }
  }

  void apply_config(ApplicationConfig config)
  {
    edited_history_length_ = config.history_length;
    {
      std::unique_lock<std::mutex> lock(data_source_queue_mutex_);
      for (auto plot : config.plots) {
        for (auto source: plot.sources) {
          uninitialized_data_source_queue_.push_back(source);
        }
      }
    }
    add_topics_from_queue();
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

  void add_topics_from_queue()
  {
    std::unique_lock<std::mutex> lock(data_source_queue_mutex_);
    for (auto queue_it = uninitialized_data_source_queue_.begin();
      queue_it != uninitialized_data_source_queue_.end(); )
    {
      auto resolved_topic = node_->get_node_topics_interface()->resolve_topic_name(
        queue_it->topic_name);
      auto type_it = available_topics_to_types_.find(resolved_topic);
      if (type_it != available_topics_to_types_.end()) {
        // if type is known, initialize and remove from queue
        const auto & itsp_it = message_type_to_introspection_.find(type_it->second);
        if (itsp_it != message_type_to_introspection_.end()) {
          MessageMember member;
          auto member_info_opt = itsp_it->second->get_member_info(queue_it->member_path);
          if (member_info_opt.has_value()) {
            member.path = queue_it->member_path;
            member.info = member_info_opt.value();
            node_->add_topic_field(resolved_topic, itsp_it->second, member);
          } else {
            // TODO(ZeilingerM) show error and warning on GUI, don't crash
            throw std::invalid_argument("could not find member in message type");
          }
        }
        queue_it = uninitialized_data_source_queue_.erase(queue_it);
      } else {
        // if type is not known yet, wait for next graph event
        // TODO(ZeilingerM) add timeout to wait for topics and types specific in configuration
        //                  display error on GUI after timeout
        ++queue_it;
      }
    }
  }

  void save_application_config()
  {
    if (active_config_file_.has_parent_path()) {
      fs::create_directories(active_config_file_.parent_path());
    }
    save_config(config_, active_config_file_);
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
        for (auto it = introspection->begin_member_infos();
          it != introspection->end_member_infos(); ++it, ++i)
        {
          std::string member_formatted = boost::algorithm::join(it->path, ".");
          ImGui::Selectable(member_formatted.c_str(), false);
          if (ImGui::IsItemHovered()) {
            ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);

            if (GImGui->HoveredIdTimer > 0.5) {
              ImGui::SetTooltip("add %s to plot 0", member_formatted.c_str());
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
        }
      } else { /* not type_available */
        ImGui::PopStyleColor();

        // color was eye-droppert' from Rviz display warnings
        // TODO(ZeilingerM) single hardcoded color is of course not robust against theme changes
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

  void update()
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
        }
        load_introspection_or_insert_missing(new_type);
      }
      add_topics_from_queue();
    }

    if (ImGui::Begin("topic list")) {
      if (ImGui::CollapsingHeader("active topics", ImGuiTreeNodeFlags_DefaultOpen)) {
        for (const auto & [topic, subscription] : node_->topics_to_subscriptions) {
          rcpputils::assert_true(
            node_->topics_to_subscriptions.find(
              topic) != node_->topics_to_subscriptions.end(),
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
      if (ImGui::CollapsingHeader("available topics", ImGuiTreeNodeFlags_DefaultOpen)) {
        for (const auto & [topic, type] : available_topics_to_types_) {
          if (node_->topics_to_subscriptions.find(topic) == node_->topics_to_subscriptions.end()) {
            if (TopicEntry(topic, type)) {
              EndTopicEntry();
            }
          }
        }
      }
    }
    ImGui::End();

    auto t = node_->now();
    auto history_dur = rclcpp::Duration::from_seconds(config_.history_length);

    auto plot_opts = PlotViewOptions {
      .use_sim_time = node_->get_parameter("use_sim_time").as_bool(),
      .t_start = t - history_dur,
      .t_end = t,
      .topics_with_time_reference_issues = {},
    };
    auto start_sec = plot_opts.t_start.seconds();
    auto end_sec = plot_opts.t_end.seconds();

    for (auto & [topic, subscription] : node_->topics_to_subscriptions) {
      for (auto & [_, buffer] : subscription.buffers_) {

        //
        // Check received data for two cases of time reference issues.
        //
        // 1) The plotting tool runs with real time reference, but data is published in
        //    simulation time. In this case the buffer would almost certainly be cleared,
        //    completely, because the start timestamp will be further in the future than the
        //    last data timestamp.
        bool time_reference_issue_likely = false;
        bool time_reference_issue_disproven = false;
        bool had_data = !buffer.empty();
        buffer.clear_data_up_to(plot_opts.t_start);
        if (had_data) {
          if (buffer.empty()) {
            time_reference_issue_likely = buffer.empty();
          } else {
            time_reference_issue_disproven = true;
          }
        }

        // 2) The plotting tool runs in simulation time, but data is published with real-time
        //    stamps.
        if (!time_reference_issue_likely) {
          auto data = buffer.data();
          time_reference_issue_likely = std::all_of(
            data.begin(), data.end(),
            [start_sec, end_sec](const ImPlotPoint & item) {
              return item.x < start_sec || item.x > end_sec;
            });
        }
        if (time_reference_issue_likely) {
          topics_with_time_reference_issues_.insert(topic);
        } else if (time_reference_issue_disproven) {
          topics_with_time_reference_issues_.erase(topic);
        }
      }
    }
    plot_opts.topics_with_time_reference_issues = topics_with_time_reference_issues_;

    for (size_t i = 0; i < config_.plots.size(); i++) {
      auto id = "plot" + std::to_string(i);
      auto & plot_config = config_.plots[i];
      if (ImGui::Begin(id.c_str())) {
        if (PlotView(id.c_str(), node_, plot_config, plot_opts)) {
          if (ImPlot::BeginDragDropTarget()) {
            if (const ImGuiPayload * payload = ImGui::AcceptDragDropPayload("topic_name")) {
              auto topic_name = std::string(static_cast<char *>(payload->Data));
              MemberPayload member_payload {
                .topic_name = topic_name.c_str(),
                .member_index = 1,
              };
              accept_member_payload(plot_config, ImPlotYAxis_1, &member_payload);
            }
            if (const ImGuiPayload * payload = ImGui::AcceptDragDropPayload("topic_member")) {
              accept_member_payload(
                plot_config, ImPlotYAxis_1,
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
                accept_member_payload(plot_config, axis, &member_payload);
              }
              if (const ImGuiPayload * payload = ImGui::AcceptDragDropPayload("topic_member")) {
                accept_member_payload(
                  plot_config, axis,
                  static_cast<MemberPayload *>(payload->Data));
              }
              ImPlot::EndDragDropTarget();
            }
          }
          EndPlotView();
        }
      }
      ImGui::End();
    }
  }

  void activate_topic_field(MemberPayload payload)
  {
    if (config_.plots.empty()) {
      auto & plot_config = config_.plots.emplace_back();
      plot_config.axes = {AxisConfig{.y_min = -1, .y_max = 1}};
    }
    accept_member_payload(config_.plots[0], ImPlotYAxis_1, &payload);
  }

  void accept_member_payload(PlotConfig & plot_config, ImPlotYAxis axis, MemberPayload * payload)
  {
    auto message_type = available_topics_to_types_.at(payload->topic_name);
    auto introspection = message_type_to_introspection_.at(message_type);
    auto member_infos = introspection->begin_member_infos();
    std::advance(member_infos, payload->member_index);
    node_->add_topic_field(payload->topic_name, introspection, *member_infos);
    plot_config.sources.insert(
      DataSourceConfig {
        .topic_name = payload->topic_name,
        .member_path = member_infos->path,
        .axis = axis,
      });
  }
};

} // namespace quickplot
