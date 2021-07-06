#include <Mahi/Gui.hpp>
#include <Mahi/Util.hpp>
#include <mutex>
#include <deque>
#include <string>
#include <utility>
#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>
#include "quickplot/typesupport_helpers.hpp"

using mahi::gui::Application;
using std::placeholders::_1;

namespace quickplot
{

struct TopicData
{
  rclcpp::Time t;
  double value;
};

class BufferNode : public rclcpp::Node
{
public:
  BufferNode()
  : Node("quickplot")
  {
  }

  std::mutex data_mutex_;
  std::deque<TopicData> data_queue_;

  void take(std::vector<double> & times, std::vector<double> & values)
  {
    std::unique_lock<std::mutex> lock(data_mutex_);
    while (!data_queue_.empty()) {
      auto data = data_queue_.front();
      times.push_back(data.t.seconds());
      values.push_back(data.value);
      data_queue_.pop_front();
    }
  }

private:
  // void data_callback(const FourWheelSteeringStamped::SharedPtr msg)
  // {
  //   TopicData data;
  //   data.t = rclcpp::Time(msg->header.stamp);
  //   data.value = msg->data.speed;
  //   {
  //     std::unique_lock<std::mutex> lock(data_mutex_);
  //     data_queue_.push_back(data);
  //   }
  // }

  // rclcpp::Subscription<FourWheelSteeringStamped>::SharedPtr subscription_;
};

class QuickPlot : public Application
{
private:
  std::shared_ptr<BufferNode> node_;
  rclcpp::Event::SharedPtr graph_event_;

  std::vector<double> times_;
  std::vector<double> data_;

  std::vector<double> history_tick_values_;
  std::vector<std::string> history_tick_labels_;

  std::vector<std::pair<std::string, std::string>> topics_;
  std::vector<std::string> active_topics_;

public:
  explicit QuickPlot(std::shared_ptr<BufferNode> _node)
  : Application(640, 480, "QuickPlot"), node_(_node)
  {
    set_vsync(true);
    ImGui::DisableViewports();
    ImGui::EnableDocking();
    graph_event_ = node_->get_graph_event();
  }

  void add_topic(std::string topic_name, std::string topic_type)
  {
    active_topics_.push_back(topic_name);

    auto introspection_library = get_typesupport_library(
      topic_type,
      rosidl_typesupport_introspection_cpp::typesupport_identifier);
    auto introspection_support = get_typesupport_handle(
      topic_type,
      rosidl_typesupport_introspection_cpp::typesupport_identifier,
      introspection_library);

    // auto support_library = get_typesupport_library(
    //   topic_type,
    //   rosidl_typesupport_cpp::typesupport_identifier);
    // auto type_support = get_typesupport_handle(
    //   topic_type,
    //   rosidl_typesupport_cpp::typesupport_identifier,
    //   support_library);
      
    // const auto* members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(introspection_support->data);
    // if (message_type_has_header(introspection_support)) {

    // }

    auto subscription = node_->create_generic_subscription(
      topic_name,
      topic_type,
      rclcpp::SensorDataQoS(),
      [this, topic_name](std::shared_ptr<rclcpp::SerializedMessage> message) {
        auto serialized_message = message->get_rcl_serialized_message();
      });
  }

  void update() override
  {
    node_->take(times_, data_);

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
    auto vec2 = get_window_size();
    ImGui::SetNextWindowPos({0, 0}, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(vec2.x, vec2.y), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowBgAlpha(0.0f);

    if (!ImGui::Begin("QuickPlot", nullptr, ImGuiWindowFlags_NoTitleBar)) {
      // Early out if the window is collapsed, as an optimization.
      ImGui::End();
      return;
    }
    ImGui::PopStyleVar();

    static ImGuiDockNodeFlags dockspace_flags = ImGuiDockNodeFlags_PassthruCentralNode |
      ImGuiWindowFlags_NoBackground;
    ImGuiID dockspace_id = ImGui::DockSpaceOverViewport(ImGui::GetMainViewport());
    ImGui::SetNextWindowDockID(dockspace_id, ImGuiCond_FirstUseEver);

    if (graph_event_->check_and_clear()) {
      topics_.clear();
      auto topics_and_types = node_->get_topic_names_and_types();
      for (const auto & pair : topics_and_types) {
        if (pair.second.size() != 1) {
          std::cerr << "Topic " << pair.first << " has multiple types, quickplot will ignore it" <<
            std::endl;
          continue;
        }
        topics_.push_back(std::make_pair(pair.first, pair.second[0]));
      }
    }

    for (const auto & topic : topics_) {
      if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_None)) {
        ImGui::SetDragDropPayload("topic_name", topic.first.c_str(), topic.first.size());
        // ImPlot::ItemIcon(dnd[k].Color); ImGui::SameLine();
        ImGui::Text(topic.first.c_str());
        ImGui::EndDragDropSource();
      }
    }

    static float history_sec = 10.0f;
    ImGui::SliderFloat("history", &history_sec, 1, 30, "%.1f s");
    auto history_dur = rclcpp::Duration::from_seconds(history_sec);
    auto t = node_->now();
    auto end_sec = t.seconds();
    auto start_sec = (t - history_dur).seconds();

    ImPlot::SetNextPlotLimitsX(start_sec, end_sec, ImGuiCond_Always);
    ImPlot::SetNextPlotLimitsY(-2, 2);

    size_t n_ticks = history_sec;
    if (n_ticks != history_tick_values_.size()) {
      history_tick_labels_.clear();
      history_tick_values_.clear();
      for (int i = n_ticks; i >= 0; i--) {
        history_tick_labels_.push_back(std::to_string(-i));
        history_tick_values_.push_back(end_sec - i);
      }
    }

    std::vector<char const *> tick_cstrings;
    tick_cstrings.reserve(history_tick_labels_.size());
    for (const auto & labels : history_tick_labels_) {
      tick_cstrings.push_back(const_cast<char *>(labels.c_str()));
    }
    ImPlot::SetNextPlotTicksX(
      history_tick_values_.data(),
      history_tick_values_.size(), tick_cstrings.data());

    if (ImPlot::BeginPlot(
        "speed", "t (header.stamp sec)", "m/s", ImVec2(-1, -1), ImPlotFlags_None))
    {
      for (int y = 0; y < 3; ++y) {
        if (ImPlot::BeginDragDropTargetY(y)) {
          if (const ImGuiPayload * payload = ImGui::AcceptDragDropPayload("topic_name")) {
            auto topic_name = std::string(static_cast<char *>(payload->Data));
          }
          ImPlot::EndDragDropTarget();
        }
      }
      for (const auto & topic : active_topics_) {
        ImPlot::PlotLine(
          topic.c_str(), times_.data(), data_.data(), times_.size());
      }
      ImPlot::EndPlot();
    }
    ImGui::End();
  }
};

} // namespace quickplot

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<quickplot::BufferNode>();
  std::thread ros_thread([ = ] {
      rclcpp::spin(node);
    });
  quickplot::QuickPlot app(node);
  app.run();
  return EXIT_SUCCESS;
}
