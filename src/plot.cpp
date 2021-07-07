#include <Mahi/Gui.hpp>
#include <Mahi/Util.hpp>
#include <mutex>
#include <deque>
#include <string>
#include <utility>
#include <memory>
#include <vector>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>
#include <rosidl_typesupport_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>
#include "quickplot/typesupport_helpers.hpp"
#include "quickplot/parser.hpp"
#include <boost/circular_buffer.hpp>

using mahi::gui::Application;
using std::placeholders::_1;
using std::vector;
using std::string;

namespace quickplot
{

struct MessageData
{
  rclcpp::Time t;
  std::vector<double> value;
};

class TopicBuffer
{
private:
  std::string topic_type_;
  std::shared_ptr<rcpputils::SharedLibrary> type_support_library_;
  std::shared_ptr<rcpputils::SharedLibrary> introspection_support_library_;
  const rosidl_message_type_support_t * type_support_handle_;
  const rosidl_message_type_support_t * introspection_support_handle_;

  std::vector<uint8_t> deserialized_message_buffer_;
  MessageDataBuffer data_buffer_;

public:
  rclcpp::GenericSubscription::SharedPtr subscription;
  std::vector<MessageData> received_data_;

  explicit TopicBuffer(std::string topic_type)
  : topic_type_(topic_type)
  {
    type_support_library_ = quickplot::get_typesupport_library(
      topic_type,
      rosidl_typesupport_cpp::typesupport_identifier);
    type_support_handle_ = quickplot::get_typesupport_handle(
      topic_type,
      rosidl_typesupport_cpp::typesupport_identifier,
      type_support_library_);

    introspection_support_library_ = quickplot::get_typesupport_library(
      topic_type_,
      rosidl_typesupport_introspection_cpp::typesupport_identifier);
    introspection_support_handle_ = quickplot::get_typesupport_handle(
      topic_type_,
      rosidl_typesupport_introspection_cpp::typesupport_identifier,
      introspection_support_library_);

    auto members =
      static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
      introspection_support_handle_
      ->data);
    deserialized_message_buffer_.resize(members->size_of_);
    members->init_function(
      deserialized_message_buffer_.data(),
      rosidl_runtime_cpp::MessageInitialization::ALL);
  }

  ~TopicBuffer()
  {
    auto members =
      static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
      introspection_support_handle_
      ->data);
    members->fini_function(deserialized_message_buffer_.data());
  }

  TopicBuffer & operator=(TopicBuffer && other) = delete;

  void add_field(std::vector<std::string> member_path)
  {
    data_buffer_.data.push_back(
      {
        member_path,
        0.0
      });
  }

  void receive(std::shared_ptr<rclcpp::SerializedMessage> message)
  {
    // TODO(ZeilingerM) not synchronized with container
    quickplot::MessageDataBuffer buffer;
    auto & linear_x_data = buffer.data.emplace_back();
    linear_x_data.member_path = {"twist", "linear", "x"};

    quickplot::parse_generic_message(
      type_support_handle_, introspection_support_handle_,
      *message, deserialized_message_buffer_.data(), data_buffer_);

    received_data_.push_back(
      {
        data_buffer_.stamp,
        {data_buffer_.data[0].value},
      });
  }
};

class BufferNode : public rclcpp::Node
{
public:
  BufferNode()
  : Node("quickplot")
  {
  }

  mutable std::mutex topic_mutex_;
  mutable std::mutex data_mutex_;
  std::unordered_map<std::string, TopicBuffer> topic_buffers_;

  void add_topic_field(
    std::string topic, std::string topic_type,
    std::vector<std::string> member_path)
  {
    std::unique_lock<std::mutex> lock(topic_mutex_);
    auto it = topic_buffers_.try_emplace(topic, topic_type);
    auto & buffer = it.first->second;
    buffer.add_field(member_path);
    if (it.second) {
      // only create subscription if entry was added first
      buffer.subscription = this->create_generic_subscription(
        topic, topic_type, rclcpp::SensorDataQoS(),
        std::bind(&TopicBuffer::receive, &buffer, _1));
    }
  }

  std::vector<std::string> active_topics()
  {
    std::unique_lock<std::mutex> lock(topic_mutex_);
    std::vector<std::string> topics;
    topics.reserve(topic_buffers_.size());
    for (const auto & kv : topic_buffers_) {
      topics.push_back(kv.first);
    }
    return topics;
  }

  std::vector<MessageData> take(std::string topic)
  {
    std::unique_lock<std::mutex> lock(data_mutex_);
    auto it = topic_buffers_.find(topic);
    if (it == topic_buffers_.end()) {
      throw std::invalid_argument("Topic " + topic + " not active");
    }
    auto copy = std::vector(it->second.received_data_);
    it->second.received_data_.clear();
    return copy;
  }
};

struct PlotData
{
  std::vector<double> times;
  std::vector<std::vector<double>> values;
};

class QuickPlot : public Application
{
private:
  std::shared_ptr<BufferNode> node_;
  rclcpp::Event::SharedPtr graph_event_;

  std::vector<double> history_tick_values_;
  std::vector<std::string> history_tick_labels_;

  std::unordered_map<std::string, std::string> available_topics_to_types_;
  std::unordered_map<std::string, PlotData> active_topics_to_data_;

public:
  explicit QuickPlot(std::shared_ptr<BufferNode> _node)
  : Application(1024, 768, "QuickPlot"), node_(_node)
  {
    set_vsync(true);
    ImGui::DisableViewports();
    ImGui::EnableDocking();
    graph_event_ = node_->get_graph_event();
    graph_event_->set(); // set manually to trigger initial topics query
  }

  void update() override
  {
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

    // static ImGuiDockNodeFlags dockspace_flags = ImGuiDockNodeFlags_PassthruCentralNode |
    //   ImGuiWindowFlags_NoBackground;
    ImGuiID dockspace_id = ImGui::DockSpaceOverViewport(ImGui::GetMainViewport());
    ImGui::SetNextWindowDockID(dockspace_id, ImGuiCond_FirstUseEver);

    if (graph_event_->check_and_clear()) {
      RCLCPP_INFO(node_->get_logger(), "Graph event checked, reloading available topics");
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
      }
    }

    {
      ImGui::BeginChild("left pane", ImVec2(150, 0), true, ImGuiWindowFlags_NoMove);
      for (const auto & topic : available_topics_to_types_) {
        ImGui::Text("%s", topic.first.c_str());
        if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_SourceAllowNullID)) {
          ImGui::SetDragDropPayload("topic_name", topic.first.c_str(), topic.first.size());
          // ImPlot::ItemIcon(dnd[k].Color); ImGui::SameLine();
          ImGui::Text("Draggin %s", topic.first.c_str());
          ImGui::EndDragDropSource();
        }
      }
      ImGui::EndChild();
    }
    ImGui::SameLine();
    {
      ImGui::BeginChild(
        "plot view", ImVec2(
          0,
          -ImGui::GetFrameHeightWithSpacing()), ImGuiWindowFlags_NoMove);
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
        for (const auto & topic : node_->active_topics()) {
          auto new_data = node_->take(topic);
          auto [entry, _] = active_topics_to_data_.try_emplace(topic);
          auto & [plot_topic, plot_data] = *entry;

          plot_data.values.resize(1);

          for (const auto & item : new_data) {
            plot_data.times.push_back(item.t.seconds());
            for (size_t i = 0; i < item.value.size(); i++) {
              plot_data.values[0].push_back(item.value[i]);
            }
          }

          for (size_t i = 0; i < plot_data.values.size(); i++) {
            ImPlot::PlotLine(
              topic.c_str(), plot_data.times.data(), plot_data.values[i].data(),
              plot_data.times.size());
          }
        }
        if (ImPlot::BeginDragDropTarget()) {
          if (const ImGuiPayload * payload = ImGui::AcceptDragDropPayload("topic_name")) {
            auto topic_name = std::string(static_cast<char *>(payload->Data));
            node_->add_topic_field(
              topic_name, available_topics_to_types_[topic_name], {"data",
                "speed"});
          }
          ImPlot::EndDragDropTarget();
        }
        ImPlot::EndPlot();
      }
      ImGui::EndChild();
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
