#include <mutex>
#include <string>
#include <utility>
#include <memory>
#include <algorithm>
#include <map>
#include <list>
#include <vector>
#include <set>
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

static DataSourceConfig source_to_config(const ActiveDataSource & source)
{
  DataSourceConfig config;
  config.topic_name = source.subscription->topic_name();
  config.member_path = to_descriptor(source.member);
  return config;
}

static TimeSeriesConfig series_to_config(const std::pair<TimeSeries, int> & p)
{
  TimeSeriesConfig config;
  const auto & [series, axis] = p;
  auto active = std::get_if<ActiveDataSource>(&series.source);
  if (active) {
    config.source = source_to_config(*active);
  }
  auto stddev_active = std::get_if<ActiveDataSource>(&series.stddev_source);
  if (stddev_active) {
    config.stddev_source = source_to_config(*stddev_active);
  }
  config.axis = axis;
  return config;
}

static PlotConfig plot_to_config(const Plot & plot)
{
  PlotConfig config;
  std::transform(
    plot.series.begin(), plot.series.end(), std::inserter(config.series, config.series.begin()),
    &series_to_config);
  config.axes = plot.axes;
  return config;
}

// construct id of a time series based on an unresolved member path
std::string series_id(const std::string & topic, const MemberSequencePathDescriptor & members)
{
  std::stringstream ss;
  ss << topic << "/" << members;
  return ss.str();
}

// construct id of a time series based on a resolved member path
// should match the id based on the unresolved path, to correctly identify a series on the GUI
// across the initialization process
std::string series_id(const std::string & topic, const MemberSequencePath & members)
{
  std::stringstream ss;
  ss << topic << "/" << members;
  return ss.str();
}

struct MessageTypeError
{
  std::string message_type;
  std::string error_message;
};

using MessageIntrospectionPtr = std::shared_ptr<MessageIntrospection>;

// either a loaded and initialized introspection service, information on why it isn't available
using MessageTypeInfo = std::variant<MessageIntrospectionPtr, MessageTypeError>;

const char * get_message_type(const MessageTypeInfo & type_info)
{
  return std::visit(
    overloaded {
      [](const std::shared_ptr<MessageIntrospection> & introspection) {
        return introspection->message_type();
      },
      [](const MessageTypeError & error) {
        return error.message_type.c_str();
      }
    }, type_info);
}

class IntrospectionCache
{
private:
  std::unordered_map<std::string, MessageIntrospectionPtr> cache_;

public:
  IntrospectionCache() = default;

  MessageIntrospectionPtr load(const std::string & message_type)
  {
    auto it = cache_.find(message_type);
    if (it != cache_.end()) {
      return it->second;
    }
    const auto & [new_entry, _] = cache_.emplace(
      message_type,
      std::make_shared<MessageIntrospection>(message_type));
    return new_entry->second;
  }
};

using TopicTypeMap = std::map<std::string, MessageTypeInfo>;

class Application
{
private:
  std::shared_ptr<QuickPlotNode> node_;
  rclcpp::Event::SharedPtr graph_event_;
  rclcpp::JumpHandler::SharedPtr jump_handler_;

  // maps already-resolved full topic names to a single ROS type
  // assumes that all publishers on this topic publish the same message type
  TopicTypeMap available_topics_to_types_;

  IntrospectionCache introspection_cache_;

  // length of history of all plots
  double history_length_;

  // list of plots to display
  std::vector<Plot> plots_;

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
    clear();
  }

  SourceDescriptor source_from_config(const DataSourceConfig & config) const
  {
    auto resolved_topic_name =
      node_->get_node_topics_interface()->resolve_topic_name(config.topic_name);
    return SourceDescriptor {
      .resolved_topic_name = resolved_topic_name,
      .member_path = config.member_path,
      .error = DataSourceError::None,
    };
  }

  std::pair<TimeSeries, int> series_from_config(const TimeSeriesConfig & config) const
  {
    TimeSeries series;
    auto source = source_from_config(config.source);
    series.id = series_id(source.resolved_topic_name, source.member_path);
    series.source = source;
    if (config.stddev_source.has_value()) {
      series.stddev_source = source_from_config(config.stddev_source.value());
    }
    return {series, config.axis};
  }

  Plot plot_from_config(const PlotConfig & config) const
  {
    Plot plot;
    std::transform(
      config.series.begin(), config.series.end(), std::back_inserter(
        plot.series), std::bind(&Application::series_from_config, this, std::placeholders::_1));
    plot.axes = config.axes;

    int max_axis = 0;
    for (const auto & [_, axis] : plot.series) {
      if (axis > max_axis) {
        max_axis = axis;
      }
    }
    plot.axes.resize(
      static_cast<size_t>(max_axis + 1), AxisConfig {
        .y_min = -1.0,
        .y_max = 1.0,
      });
    return plot;
  }

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

  void apply_config(ApplicationConfig config)
  {
    plots_.clear();
    std::transform(
      config.plots.begin(), config.plots.end(), std::back_inserter(plots_),
      std::bind(&Application::plot_from_config, this, std::placeholders::_1));
    history_length_ = config.history_length;
    initialize_pending_sources();
  }

  ApplicationConfig get_config() const
  {
    ApplicationConfig config;
    config.history_length = history_length_;
    std::transform(
      plots_.begin(), plots_.end(), std::back_inserter(config.plots),
      &plot_to_config);
    return config;
  }

  std::optional<ActiveDataSource> try_initialize_source(
    SourceDescriptor & descriptor)
  {
    auto topic = descriptor.resolved_topic_name;
    auto type_it = available_topics_to_types_.find(topic);
    if (type_it != available_topics_to_types_.end()) {
      // if type is known, initialize and set ready state
      const auto & [topic, type_info] = *type_it;
      auto introspection_opt = std::get_if<MessageIntrospectionPtr>(&type_info);
      if (introspection_opt) {
        auto introspection = *introspection_opt;
        try {
          auto member_opt = introspection->get_member_sequence_path(descriptor.member_path);
          if (member_opt.has_value()) {
            auto subscription = node_->get_or_create_subscription(topic, introspection);
            auto member = member_opt.value();
            auto buffer = subscription->add_source(member);
            return ActiveDataSource {
              .warning = DataWarning::None,
              .subscription = subscription,
              .member = member,
              .data = buffer,
            };
          } else {
            descriptor.error = DataSourceError::InvalidMember;
          }
        } catch (const introspection_error &) {
          descriptor.error = DataSourceError::InvalidMember;
        }
      } else {
        descriptor.error = DataSourceError::MessageTypeUnavailable;
      }
    } // else {
      // if type is not known yet, we leave the source in an uninitialized state, and the GUI should
      // show the source as unavailable
    // }
    return std::nullopt;
  }

  void ensure_series_initialized(TimeSeries & series)
  {
    auto descriptor = std::get_if<SourceDescriptor>(&series.source);
    if (descriptor && descriptor->error == DataSourceError::None) {
      auto active_opt = try_initialize_source(*descriptor);
      if (active_opt.has_value()) {
        series.source = active_opt.value();
      }
    }

    auto stddev_descriptor = std::get_if<SourceDescriptor>(&series.stddev_source);
    if (stddev_descriptor && stddev_descriptor->error == DataSourceError::None) {
      auto active_opt = try_initialize_source(*stddev_descriptor);
      if (active_opt.has_value()) {
        series.stddev_source = active_opt.value();
      }
    }
  }

  void initialize_pending_sources()
  {
    for (auto & plot : plots_) {
      for (auto & [series, _] : plot.series) {
        ensure_series_initialized(series);
      }
    }
  }

  bool TopicEntryError(const std::string & topic, const MessageTypeError & error)
  {
    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyle().Colors[ImGuiCol_TextDisabled]);
    if (ImGui::TreeNodeEx(topic.c_str(), ImGuiTreeNodeFlags_Leaf)) {
      ImGui::PopStyleColor();
      // color was eye-dropped from Rviz display warnings
      // TODO(ZeilingerM) single hardcoded color does not work well with theme changes
      static ImVec4 WARNING_COLOR = static_cast<ImVec4>(ImColor::HSV(0.1083f, 0.968f, 0.867f));
      ImGui::PushStyleColor(ImGuiCol_Text, WARNING_COLOR);
      ImGui::TextWrapped("message type '%s' is not available:", error.message_type.c_str());
      ImGui::TextWrapped("%s", error.error_message.c_str());
      ImGui::PopStyleColor();
      return true;
    }
    ImGui::PopStyleColor();
    return false;
  }

  bool TopicEntryInitialized(
    const std::string & topic,
    const std::shared_ptr<MessageIntrospection> & introspection)
  {
    if (ImGui::TreeNode(topic.c_str())) {
      if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_SourceAllowNullID)) {
        ImGui::SetDragDropPayload("topic_name", topic.c_str(), topic.size());
        ImPlot::ItemIcon(ImGui::GetColorU32(ImGuiCol_Text));
        ImGui::SameLine();
        ImGui::Text("Dragging %s", topic.c_str());
        ImGui::EndDragDropSource();
      }
      size_t i {0};
      for (const auto & member : introspection->members()) {
        if (is_numeric(member.back()->type_id_) && !contains_sequence(member)) {
          std::stringstream ss;
          ss << member;
          auto member_str = ss.str();
          ImGui::Selectable(member_str.c_str(), false);
          MemberPayload payload {
            .topic_name = topic.c_str(),
            .member = assume_members_unindexed(member),
          };
          if (ImGui::IsItemHovered()) {
            ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);

            if (GImGui->HoveredIdTimer > 0.5) {
              ImGui::BeginTooltip();
              ImGui::Text("add %s to plot0", member_str.c_str());
              ImGui::Text("or drag and drop on plot of choice");
              ImGui::EndTooltip();
            }
            if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
              add_topic_field_to_plot(payload);
            }
          }
          if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_SourceAllowNullID)) {
            ImGui::SetDragDropPayload("topic_member", &payload, sizeof(payload));
            ImGui::EndDragDropSource();
          }
        }
        ++i;
      }
      return true;
    }
    return false;
  }

  bool TopicEntry(const std::string & topic, const MessageTypeInfo & type_info)
  {
    return std::visit(
      overloaded {
        [this, topic](const std::shared_ptr<MessageIntrospection> & introspection) {
          return TopicEntryInitialized(topic, introspection);
        },
        [this, topic](const MessageTypeError & error) {
          return TopicEntryError(topic, error);
        }
      }, type_info);
  }

  void EndTopicEntry()
  {
    ImGui::TreePop();
  }

  void update_topics()
  {
    if (graph_event_->check_and_clear()) {
      auto topics_and_types = node_->get_topic_names_and_types();
      for (const auto & [topic, types] : topics_and_types) {
        if (types.size() != 1) {
          std::cerr << "topic " << topic << " has multiple types and will be ignored" <<
            std::endl;
          continue;
        }
        auto new_type = types[0];
        auto it = available_topics_to_types_.find(topic);
        if (it != available_topics_to_types_.end()) {
          const auto & [_, type_info] = *it;
          // the topic was already in the map, so we possibly have active listeners
          // parsing a message type that may have changed
          if (new_type.compare(get_message_type(type_info)) != 0) {
            std::invalid_argument("type of topic " + topic + " changed");
          }
        } else {
          // for new topics, load their introspection support
          try {
            auto introspection = introspection_cache_.load(new_type);
            available_topics_to_types_.emplace(topic, introspection);
          } catch (const introspection_error & e) {
            available_topics_to_types_.emplace(
              topic, MessageTypeError {
                .message_type = new_type,
                .error_message = e.what(),
              });
          }
        }
      }
      initialize_pending_sources();
    }
  }

  std::optional<DataWarning> prune_and_detect_clock_issues(
    PlotDataBuffer & buffer,
    const PlotViewOptions & plot_opts)
  {
    auto start_sec = plot_opts.t_start.seconds();
    auto end_sec = plot_opts.t_end.seconds();
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
        data->begin(), data->end(),
        [start_sec, end_sec](const ImPlotPoint & item) {
          return item.x < start_sec || item.x > end_sec;
        });
    }
    if (clock_issue_likely) {
      return DataWarning::TimeStampOutOfRange;
    } else if (clock_issue_disproven) {
      return DataWarning::None;
    }
    return std::nullopt;
  }

  void update_data_source(DataSource & source, const PlotViewOptions & plot_opts)
  {
    auto active = std::get_if<ActiveDataSource>(&source);
    if (active) {
      auto new_warning = prune_and_detect_clock_issues(*active->data, plot_opts);
      if (new_warning.has_value()) {
        active->warning = new_warning.value();
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

    // prune all data to time window of plot
    for (auto & plot : plots_) {
      for (auto & [series, _] : plot.series) {
        update_data_source(series.source, plot_opts);
        update_data_source(series.stddev_source, plot_opts);
      }
    }
    PlotDock(plot_opts);
  }

  void TopicList()
  {
    ImGuiWindowFlags list_window_flags = ImGuiWindowFlags_None;
    if (ImGui::Begin(TOPIC_LIST_WINDOW_ID, nullptr, list_window_flags)) {
      // accumulate and sort subscriptions of active plots
      auto cmp_topic_names =
        [](std::shared_ptr<PlotSubscription> s1, std::shared_ptr<PlotSubscription> s2) {
          return s1->topic_name() < s2->topic_name();
        };
      std::set<std::shared_ptr<PlotSubscription>, decltype(cmp_topic_names)> active_topics(
        cmp_topic_names);
      for (auto & plot : plots_) {
        for (auto & [series, _] : plot.series) {
          auto active = std::get_if<ActiveDataSource>(&series.source);
          if (active) {
            active_topics.insert(active->subscription);
          }
        }
      }

      // display list of subscribed topics and their receive stats
      if (!active_topics.empty()) {
        if (ImGui::CollapsingHeader("active topics", ImGuiTreeNodeFlags_DefaultOpen)) {
          for (const auto & subscription : active_topics) {
            auto type_it = available_topics_to_types_.find(subscription->topic_name());
            rcpputils::assert_true(
              type_it != available_topics_to_types_.end(),
              "topics can only become active when their type is known");
            if (TopicEntry(subscription->topic_name(), type_it->second)) {
              auto stats = subscription->receive_period_stats();
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

      // display filtered list of topics; include those that can't be subscribed with a warning
      if (ImGui::CollapsingHeader("available topics", ImGuiTreeNodeFlags_DefaultOpen)) {
        // according to https://design.ros2.org/articles/topic_and_service_names.html, max topic
        // name length is 256
        const size_t filter_text_length = 256 + 1;
        static char filter_text_raw[filter_text_length];
        ImGui::PushItemWidth(-1);
        ImGui::InputTextWithHint("", "filter topic or type", filter_text_raw, filter_text_length);
        ImGui::PopItemWidth();

        std::string filter_text(filter_text_raw);

        std::list<TopicTypeMap::iterator> shown_available_topics;
        size_t total_available = 0;
        auto it = available_topics_to_types_.begin();
        for (;it != available_topics_to_types_.end(); ++it) {
          const auto & [topic, type] = *it;
          if (node_->is_subscribed_to(topic)) {
            // skip subscribed topics, those are already listed in the 'active topics' section
            continue;
          }
          ++total_available;
          if (topic.find(filter_text) == std::string::npos) {
            continue;
          }
          shown_available_topics.push_back(it);
        }

        // defer rendering the items up to this point, to print the number of shown topics before the topics
        if (filter_text[0] != '\0') {
          // if filter is passed, show amount of filtered topics
          ImGui::PushItemWidth(-1);
          ImGui::Text("Showing %lu of %lu topics", shown_available_topics.size(), total_available);
          ImGui::PopItemWidth();
        }
        for (const auto & it : shown_available_topics) {
          if (TopicEntry(it->first, it->second)) {
            EndTopicEntry();
          }
        }
      }
    }
    ImGui::End();
  }

  void PlotDock(const PlotViewOptions & plot_opts)
  {
    auto plot_it = plots_.begin();
    for (size_t i = 0; plot_it != plots_.end(); i++) {
      auto id = "plot" + std::to_string(i);
      bool plot_window_enabled = true;

      // for plot windows, the padding is redundant with dock borders
      ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0, 0.0));
      if (ImGui::Begin(id.c_str(), &plot_window_enabled)) {
        ImGui::PopStyleVar();

        auto plot_view_result = BeginPlotView(id.c_str(), *plot_it, plot_opts);
        if (plot_view_result.displayed) {
          history_length_ += plot_view_result.time_scale_delta;

          auto payload_opt = PlotView(*plot_it, plot_opts);
          if (payload_opt.has_value()) {
            auto payload = payload_opt.value();
            accept_member_payload(
              *plot_it, payload.axis, &payload.member);
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

  void add_topic_field_to_plot(MemberPayload payload)
  {
    if (plots_.empty()) {
      auto & new_plot = plots_.emplace_back();
      new_plot.axes = {AxisConfig{.y_min = -1, .y_max = 1}};
    } else if (plots_[0].axes.empty()) {
      auto & new_axis = plots_[0].axes.emplace_back();
      new_axis.y_min = -1.0;
      new_axis.y_max = 1.0;
    }
    accept_member_payload(plots_[0], ImPlotYAxis_1, &payload);
  }

  void accept_member_payload(Plot & plot, ImPlotYAxis axis, MemberPayload * payload)
  {
    const auto& type_info = available_topics_to_types_.at(payload->topic_name);
    auto introspection_opt = std::get_if<MessageIntrospectionPtr>(&type_info);
    rcpputils::assert_true(
      static_cast<bool>(introspection_opt),
      "message type must be available when accept_member_payload is triggered");
    auto subscription = node_->get_or_create_subscription(payload->topic_name, *introspection_opt);
    auto buffer = subscription->add_source(payload->member);

    auto id = series_id(payload->topic_name, payload->member);
    auto it = std::find_if(
      plot.series.begin(), plot.series.end(), [&id](const auto & item) {
        return item.first.id == id;
      });
    if (it != plot.series.end()) {
      // ensure the same source config is not added twice
      return;
    }
    auto & [new_series, new_axis] = plot.series.emplace_back();
    new_axis = axis;
    // assume the state is Ok, since the topic was drag-dropped from the available list
    new_series.source = ActiveDataSource {
      .warning = DataWarning::None,
      .subscription = subscription,
      .member = payload->member,
      .data = buffer,
    };
    new_series.id = id;

    while (static_cast<size_t>(axis + 1) > plot.axes.size()) {
      auto & new_axis = plot.axes.emplace_back();
      new_axis.y_min = -1.0;
      new_axis.y_max = 1.0;
    }
  }

  void clear()
  {
    node_->clear();
  }
};

} // namespace quickplot
