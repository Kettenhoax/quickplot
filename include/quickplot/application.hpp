
#include <boost/algorithm/string/join.hpp>
#include <mutex>
#include <string>
#include <utility>
#include <memory>
#include <deque>
#include <list>
#include <vector>
#include <unordered_map>
#include <filesystem>
#include <rosidl_typesupport_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>
#include "quickplot/config.hpp"
#include "quickplot/node.hpp"
#include <rcpputils/asserts.hpp>

namespace fs = std::filesystem;

namespace quickplot
{
constexpr char TOPIC_NAME_PAYLOAD_LABEL[] = "quickplot_topic_name";
constexpr char FIELD_INFO_PAYLOAD_LABEL[] = "quickplot_field_info";

ImPlotPoint circular_buffer_access(void * data, int idx)
{
  auto buffer = reinterpret_cast<PlotDataCircularBuffer *>(data);
  return buffer->at(idx);
}

struct MemberPayload
{
  const char * topic_name;
  size_t member_index;
};

void print_exception_recursive(const std::exception & e, int level = 0, int after_level = 0)
{
  if (level >= after_level) {
    std::cerr << std::string(level, ' ') << e.what() << '\n';
  }
  try {
    std::rethrow_if_nested(e);
  } catch (const std::exception & e) {
    print_exception_recursive(e, level + 1, after_level);
  } catch (...) {
  }
}

class Application
{
private:
  fs::path active_config_file_;
  ApplicationConfig config_;

  std::shared_ptr<QuickPlotNode> node_;
  rclcpp::Event::SharedPtr graph_event_;
  rclcpp::JumpHandler::SharedPtr jump_handler_;

  double edited_history_length_;
  std::vector<double> history_tick_values_;
  std::vector<std::string> history_tick_labels_;

  std::unordered_map<std::string, std::string> available_topics_to_types_;
  std::unordered_map<std::string,
    std::shared_ptr<MessageIntrospection>> message_type_to_introspection_;

  // queue of requested topic data, for which the topic metadata has not been received yet
  std::mutex data_source_queue_mutex_;
  std::deque<DataSourceConfig> uninitialized_data_source_queue_;

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

      // TODO(ZeilingerM) resolve topic name on-demand rather than overwriting config
      for (auto & plot : config_.plots) {
        for (auto & source: plot.sources) {
          source.topic_name = node_->get_node_topics_interface()->resolve_topic_name(
            source.topic_name);
        }
      }
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

  void add_topics_from_queue()
  {
    std::unique_lock<std::mutex> lock(data_source_queue_mutex_);
    for (auto it = uninitialized_data_source_queue_.begin();
      it != uninitialized_data_source_queue_.end(); )
    {
      auto type_it = available_topics_to_types_.find(it->topic_name);
      if (type_it != available_topics_to_types_.end()) {
        auto introspection = message_type_to_introspection_.at(type_it->second);
        MessageMember member;
        auto member_info_opt = introspection->get_member_info(it->member_path);
        if (member_info_opt.has_value()) {
          member.path = it->member_path;
          member.info = member_info_opt.value();
          node_->add_topic_field(
            it->topic_name, introspection, member);
        } else {
          // TODO(ZeilingerM) show error and warning on GUI
          throw std::invalid_argument("Could not find member in message type");
        }
        it = uninitialized_data_source_queue_.erase(it);
      } else {
        ++it;
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

  void plot_view(size_t i, PlotConfig & plot, rclcpp::Time t_start, rclcpp::Time t_end)
  {
    auto id = "plot" + std::to_string(i);
    if (ImGui::Begin(id.c_str())) {
      for (ImPlotYAxis a = 0; a < static_cast<ImPlotYAxis>(plot.axes.size()); a++) {
        const auto & axis_config = plot.axes[a];
        ImPlot::SetNextPlotLimitsY(
          axis_config.y_min, axis_config.y_max,
          ImGuiCond_Once, a);
      }

      std::vector<char const *> tick_cstrings;
      tick_cstrings.reserve(history_tick_labels_.size());
      for (const auto & labels : history_tick_labels_) {
        tick_cstrings.push_back(const_cast<char *>(labels.c_str()));
      }

      auto start_sec = t_start.seconds();
      auto end_sec = t_end.seconds();
      ImPlot::SetNextPlotLimitsX(start_sec, end_sec, ImGuiCond_Always);
      ImPlot::SetNextPlotTicksX(
        history_tick_values_.data(),
        history_tick_values_.size(), tick_cstrings.data());

      std::stringstream x_label;
      x_label << "t (header.stamp sec)";
      if (node_->get_parameter("use_sim_time").as_bool()) {
        x_label << " use_sim_time:=true";
      }
      if (ImPlot::BeginPlot(
          id.c_str(), x_label.str().c_str(), nullptr, ImVec2(-1, -1),
          (plot.axes.size() >= 2 ? ImPlotFlags_YAxis2 : 0) |
          (plot.axes.size() >= 3 ? ImPlotFlags_YAxis3 : 0) | ImPlotFlags_NoTitle))
      {
        if (ImPlot::IsPlotXAxisHovered()) {
          ImGui::BeginTooltip();

          ImGui::Text("now: %.2f", t_end.seconds());
          ImGui::EndTooltip();
        }
        for (ImPlotYAxis a = 0; a < static_cast<ImPlotYAxis>(plot.axes.size()); a++) {
          for (auto source_it = plot.sources.begin(); source_it != plot.sources.end(); ) {
            bool removed = false;
            if (source_it->axis == a) {
              std::stringstream ss;
              ss << source_it->topic_name << " " << boost::algorithm::join(
                source_it->member_path, ".");

              ImPlot::SetPlotYAxis(a);

              std::unique_lock<std::mutex> lock(node_->topic_mutex);
              auto it = node_->topics_to_subscriptions.find(source_it->topic_name);
              std::string series_label;
              if (it == node_->topics_to_subscriptions.end()) {
                // plot empty line to show warning in legend
                ss << " (not received)";
                series_label = ss.str();
                ImPlot::HideNextItem(true, ImGuiCond_Always);
                ImPlot::PlotLine(series_label.c_str(), static_cast<float *>(nullptr), 0);
              } else {
                auto & buffer = it->second.get_buffer(source_it->member_path);
                buffer.clear_data_up_to(t_start);
                series_label = ss.str();
                {
                  std::unique_lock<std::mutex> lock(buffer.data_mutex);
                  ImPlot::PlotLineG(
                    series_label.c_str(),
                    &circular_buffer_access,
                    &buffer.data,
                    buffer.data.size());
                  if (!buffer.data.empty()) {
                    if (std::all_of(
                        buffer.data.begin(), buffer.data.end(),
                        [start_sec, end_sec](const ImPlotPoint & item) {
                          return item.x < start_sec || item.x > end_sec;
                        }))
                    {
                      // all points are outside of the time bounds
                      // the most likely cause is a sim time misconfiguration
                      auto warning_color = ImPlot::GetLastItemColor();
                      ImPlot::AnnotateClamped(
                        (end_sec - start_sec) / 2.0, 0.5, ImVec2(
                          0,
                          0), warning_color, "WARNING [%s]: all data points are outside of the x range, it may be required to set use_sim_time when launching quickplot",
                        source_it->topic_name.c_str());
                    }
                  }
                }
              }

              if (ImPlot::BeginLegendPopup(series_label.c_str())) {
                if (ImGui::Button("remove")) {
                  source_it = plot.sources.erase(source_it);
                  removed = true;
                }
                ImPlot::EndLegendPopup();
              }
            }
            if (!removed) {
              ++source_it;
            }
          }
        }
        if (ImPlot::BeginDragDropTarget()) {
          if (const ImGuiPayload * payload = ImGui::AcceptDragDropPayload("topic_name")) {
            auto topic_name = std::string(static_cast<char *>(payload->Data));
            MemberPayload member_payload {
              .topic_name = topic_name.c_str(),
              .member_index = 1,
            };
            accept_member_payload(plot, ImPlotYAxis_1, &member_payload);
          }
          if (const ImGuiPayload * payload = ImGui::AcceptDragDropPayload("topic_member")) {
            accept_member_payload(plot, ImPlotYAxis_1, static_cast<MemberPayload *>(payload->Data));
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
              accept_member_payload(plot, axis, &member_payload);
            }
            if (const ImGuiPayload * payload = ImGui::AcceptDragDropPayload("topic_member")) {
              accept_member_payload(plot, axis, static_cast<MemberPayload *>(payload->Data));
            }
            ImPlot::EndDragDropTarget();
          }
        }
        for (ImPlotYAxis a = 0; a < static_cast<ImPlotYAxis>(plot.axes.size()); a++) {
          auto & axis_config = plot.axes[a];
          // store the potentially user-defined limit
          {
            auto limits = ImPlot::GetPlotLimits(a);
            axis_config.y_min = limits.Y.Min;
            axis_config.y_max = limits.Y.Max;
          }
        }
        ImPlot::EndPlot();
      }
    }
    ImGui::End();
  }

  bool TopicEntry(const std::string & topic, const std::string & type)
  {
    if (ImGui::TreeNode(topic.c_str())) {
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
      return true;
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
          std::cerr << "Topic " << topic << " has multiple types, quickplot will ignore it" <<
            std::endl;
          continue;
        }
        auto new_type = types[0];
        auto [entry, inserted] = available_topics_to_types_.try_emplace(topic, new_type);
        if (!inserted) {
          // the topic was already in the map, so we possibly have active listeners
          // parsing a message type that may have changed
          if (entry->second != new_type) {
            std::invalid_argument("Type of topic " + topic + " changed");
          }
        }
        if (message_type_to_introspection_.find(new_type) == message_type_to_introspection_.end()) {
          message_type_to_introspection_.emplace(
            new_type,
            std::make_shared<MessageIntrospection>(new_type));
        }
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

    auto history_dur = rclcpp::Duration::from_seconds(config_.history_length);
    auto t = node_->now();
    auto end_sec = t.seconds();
    auto start = t - history_dur;

    size_t n_ticks = static_cast<size_t>(config_.history_length);
    history_tick_values_.resize(n_ticks);
    if (n_ticks != history_tick_labels_.size()) {
      history_tick_labels_.resize(n_ticks);
      for (size_t i = 0; i < n_ticks; i++) {
        history_tick_labels_[i] =
          std::to_string(-static_cast<int>(n_ticks) + static_cast<int>(i));
      }
    }
    for (size_t i = 0; i < n_ticks; i++) {
      history_tick_values_[i] = end_sec - static_cast<int>(n_ticks) + i;
    }

    for (size_t i = 0; i < config_.plots.size(); i++) {
      plot_view(i, config_.plots[i], start, t);
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
    plot_config.sources.push_back(
      DataSourceConfig {
        .topic_name = payload->topic_name,
        .member_path = member_infos->path,
        .axis = axis,
      });
  }
};

} // namespace quickplot