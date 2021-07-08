#include <Mahi/Gui.hpp>
#include <Mahi/Util.hpp>
#include <mutex>
#include <string>
#include <utility>
#include <memory>
#include <list>
#include <vector>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>
#include <rosidl_typesupport_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>
#include "quickplot/message_parser.hpp"
#include <boost/circular_buffer.hpp>

using mahi::gui::Application;
using std::placeholders::_1;

namespace quickplot
{

struct PlotData
{
  double timestamp;
  double value;
};

using PlotDataCircularBuffer = boost::circular_buffer<PlotData>;

struct PlotDataBuffer
{
  // path of member in message tree
  // for example, the path of linear.x in geometry_msgs/Twist is {"linear", "x"}
  std::vector<std::string> member_path;
  // offset of member into message memory
  MemberInfo member_info;
  std::string axis_name;

  // lock this mutex when accessing data from the plot buffer
  std::mutex data_mutex;
  PlotDataCircularBuffer data;
  size_t start_index;

  PlotDataBuffer(
    std::vector<std::string> member_path, MemberInfo member_info,
    std::string axis_name)
  : member_path(member_path), member_info(member_info), axis_name(axis_name), data_mutex(), data(100),
    start_index()
  {

  }

  void clear_data_up_to(rclcpp::Time t)
  {
    auto s = t.seconds();
    while (!data.empty()) {
      if (data.front().timestamp < s) {
        data.pop_front();
      } else {
        break;
      }
    }
  }
};

class PlotSubscription
{
private:
  std::string topic_name_;
  std::vector<uint8_t> message_buffer_;
  std::shared_ptr<IntrospectionMessageDeserializer> deserializer_;
  rclcpp::GenericSubscription::SharedPtr subscription_;

public:
  std::mutex buffers_mutex_;

  // one data buffer per plotted member of a message
  // using list instead of vector, since the emplace_back operation does not require
  // the element to be MoveConstructible for resizing the array
  std::list<PlotDataBuffer> buffers;

  explicit PlotSubscription(
    std::string topic_name,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface,
    std::shared_ptr<IntrospectionMessageDeserializer> deserializer)
  : topic_name_(topic_name), deserializer_(deserializer)
  {
    message_buffer_ = deserializer_->init_buffer();
    subscription_ = rclcpp::create_generic_subscription(
      topics_interface,
      topic_name_,
      deserializer_->topic_type(),
      rclcpp::SensorDataQoS(),
      std::bind(&PlotSubscription::receive_callback, this, _1),
      rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>()
    );
  }

  ~PlotSubscription()
  {
    deserializer_->fini_buffer(message_buffer_);
  }

  // disable copy and move
  PlotSubscription & operator=(PlotSubscription && other) = delete;

  void add_field(std::vector<std::string> member_path)
  {
    std::stringstream ss;
    ss << topic_name_;
    for (const auto & member : member_path) {
      ss << "." << member;
    }
    auto member = deserializer_->find_member(member_path);
    if (!member.has_value()) {
      throw std::invalid_argument("Member not found");
    }
    std::unique_lock<std::mutex> lock(buffers_mutex_);
    buffers.emplace_back(
      member_path,
      member.value(),
      ss.str()
    );
  }

  void receive_callback(std::shared_ptr<rclcpp::SerializedMessage> message)
  {
    deserializer_->deserialize(*message, message_buffer_.data());
    rclcpp::Time stamp = deserializer_->get_header_stamp(message_buffer_.data());
    std::unique_lock<std::mutex> lock(buffers_mutex_);
    for (auto & buffer : buffers) {
      std::unique_lock<std::mutex> lock(buffer.data_mutex);
      buffer.data.push_back(
        PlotData {
          .timestamp = stamp.seconds(),
          .value = cast_numeric(message_buffer_.data(), buffer.member_info)
        });
    }
    if (!buffers.empty()) {
      std::cout << "capacity " <<
        buffers.front().data.capacity() << std::endl;
    }
  }
};

class BufferNode : public rclcpp::Node
{
public:
  BufferNode()
  : Node("quickplot")
  {
  }

  std::mutex topic_mutex;
  std::unordered_map<std::string,
    std::shared_ptr<IntrospectionMessageDeserializer>> type_to_parsers_;
  std::unordered_map<std::string, PlotSubscription> topics_to_subscriptions;

  void add_topic_field(
    std::string topic, std::string topic_type,
    std::vector<std::string> member_path)
  {
    std::unique_lock<std::mutex> lock(topic_mutex);
    // ensure message parser is initialized
    if (type_to_parsers_.find(topic_type) == type_to_parsers_.end()) {
      type_to_parsers_.emplace(
        topic_type,
        std::make_shared<IntrospectionMessageDeserializer>(topic_type));
    }
    auto it = topics_to_subscriptions.try_emplace(
      topic,
      topic, get_node_topics_interface(), type_to_parsers_.at(topic_type));
    auto & subscription = it.first->second;
    subscription.add_field(member_path);
  }
};

ImPlotPoint circular_buffer_access(void * data, int idx)
{
  auto buffer = reinterpret_cast<PlotDataCircularBuffer *>(data);
  const auto & item = buffer->at(idx);
  return ImPlotPoint(item.timestamp, item.value);
}

class QuickPlot : public Application
{
private:
  std::shared_ptr<BufferNode> node_;
  rclcpp::Event::SharedPtr graph_event_;

  std::vector<double> history_tick_values_;
  std::vector<std::string> history_tick_labels_;

  std::unordered_map<std::string, std::string> available_topics_to_types_;
  std::unordered_map<std::string, std::string> topic_type_to_parser_;
  std::unordered_map<std::string, PlotData> active_topics_to_data_;

public:
  explicit QuickPlot(std::shared_ptr<BufferNode> _node)
  : Application(1280, 768, "QuickPlot"), node_(_node)
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
      auto start = t - history_dur;
      auto start_sec = start.seconds();

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
        for (auto & [_, subscription] : node_->topics_to_subscriptions) {
          std::unique_lock<std::mutex> lock(subscription.buffers_mutex_);
          for (auto & buffer : subscription.buffers) {
            std::unique_lock<std::mutex> lock(buffer.data_mutex);
            buffer.clear_data_up_to(start);
            ImPlot::PlotLineG(
              buffer.axis_name.c_str(),
              &circular_buffer_access,
              &buffer.data,
              buffer.data.size());
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
