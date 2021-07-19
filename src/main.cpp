#include "implot.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <mutex>
#include <string>
#include <utility>
#include <memory>
#include <deque>
#include <list>
#include <vector>
#include <unordered_map>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <rosidl_typesupport_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>
#include "quickplot/message_parser.hpp"
#include "quickplot/config.hpp"
#include <boost/circular_buffer.hpp>

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
    std::string axis_name, size_t capacity)
  : member_path(member_path), member_info(member_info), axis_name(axis_name), data_mutex(), data(
      capacity),
    start_index()
  {

  }

  void clear_data_up_to(rclcpp::Time t)
  {
    std::unique_lock<std::mutex> lock(data_mutex);
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

  void add_field(std::vector<std::string> member_path, size_t capacity)
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
      ss.str(),
      capacity
    );
  }

  void receive_callback(std::shared_ptr<rclcpp::SerializedMessage> message)
  {
    deserializer_->deserialize(*message, message_buffer_.data());
    rclcpp::Time stamp = deserializer_->get_header_stamp(message_buffer_.data());
    std::unique_lock<std::mutex> lock(buffers_mutex_);
    for (auto & buffer : buffers) {
      std::unique_lock<std::mutex> lock(buffer.data_mutex);
      if (buffer.data.full()) {
        buffer.data.resize(buffer.data.size() * 2);
      }
      buffer.data.push_back(
        PlotData {
          .timestamp = stamp.seconds(),
          .value = cast_numeric(message_buffer_.data(), buffer.member_info)
        });
    }
  }
};

class QuickPlotNode : public rclcpp::Node
{
public:
  QuickPlotNode()
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
    size_t capacity = 10; // TODO(ZeilingerM) compute capacity from expected frequencies
    subscription.add_field(member_path, capacity);
  }
};

ImPlotPoint circular_buffer_access(void * data, int idx)
{
  auto buffer = reinterpret_cast<PlotDataCircularBuffer *>(data);
  const auto & item = buffer->at(idx);
  return ImPlotPoint(item.timestamp, item.value);
}

static void glfw_error_callback(int error, const char * description)
{
  fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

class Application
{
private:
  std::shared_ptr<QuickPlotNode> node_;
  rclcpp::Event::SharedPtr graph_event_;

  double history_length_;
  std::vector<double> history_tick_values_;
  std::vector<std::string> history_tick_labels_;

  std::unordered_map<std::string, std::string> available_topics_to_types_;
  std::unordered_map<std::string, std::string> topic_type_to_parser_;
  std::unordered_map<std::string, PlotData> active_topics_to_data_;

  std::deque<TopicPlotConfig> untyped_topic_queue_;

public:
  explicit Application(std::shared_ptr<QuickPlotNode> _node)
  : node_(_node)
  {
    graph_event_ = node_->get_graph_event();
    graph_event_->set(); // set manually to trigger initial topics query

    auto config_path = get_default_config_path();
    ApplicationConfig config;
    if (std::filesystem::exists(config_path)) {
      config = load_config(config_path);
    } else {
      config = default_config();
    }
    apply_config(config);
  }

  void apply_config(ApplicationConfig config)
  {
    history_length_ = config.history_length;
    std::copy(
      config.topic_plots.begin(), config.topic_plots.end(),
      std::back_inserter(untyped_topic_queue_));
    add_topics_from_queue();
  }

  void add_topics_from_queue()
  {
    for (auto it = untyped_topic_queue_.begin(); it != untyped_topic_queue_.end(); ) {
      auto type_it = available_topics_to_types_.find(it->topic_name);
      if (type_it != available_topics_to_types_.end()) {
        for (const auto & field : it->members) {
          node_->add_topic_field(it->topic_name, type_it->second, field.path);
        }
        it = untyped_topic_queue_.erase(it);
      } else {
        ++it;
      }
    }
  }

  ApplicationConfig collect_config()
  {
    ApplicationConfig config;
    config.history_length = history_length_;
    {
      std::lock_guard<std::mutex> lock(node_->topic_mutex);
      for (const auto &[topic_name, subscription] : node_->topics_to_subscriptions) {
        auto & topic_plot = config.topic_plots.emplace_back();
        topic_plot.topic_name = topic_name;
        for (const auto & buffer : subscription.buffers) {
          topic_plot.members.push_back(
            MessageMemberPlotConfig {
              .path = buffer.member_path
            });
        }
      }
    }
    return config;
  }

  void update()
  {
    if (!ImGui::Begin("QuickPlot", nullptr, ImGuiWindowFlags_MenuBar)) {
      // Early out if the window is collapsed, as an optimization.
      ImGui::End();
      return;
    }
    if (ImGui::BeginMenuBar()) {
      if (ImGui::BeginMenu("File")) {
        if (ImGui::MenuItem("Save config", "Ctrl+S")) {
          save_config(collect_config(), get_default_config_path());
        }
        ImGui::EndMenu();
      }
      ImGui::EndMenuBar();
    }

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
      add_topics_from_queue();
    }

    {
      ImGui::BeginChild("topic list", ImVec2(150, 0), true, ImGuiWindowFlags_NoMove);
      for (const auto & topic : available_topics_to_types_) {
        ImGui::Text("%s", topic.first.c_str());
        if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_SourceAllowNullID)) {
          ImGui::SetDragDropPayload("topic_name", topic.first.c_str(), topic.first.size());
          ImPlot::ItemIcon(ImGui::GetColorU32(ImGuiCol_Text));
          ImGui::SameLine();
          ImGui::Text("Dragging %s", topic.first.c_str());
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
          -ImGui::GetFrameHeightWithSpacing()), false, ImGuiWindowFlags_NoMove);
      ImGui::InputDouble("history", &history_length_, 1, 10, "%.1f s");
      auto history_dur = rclcpp::Duration::from_seconds(history_length_);
      auto t = node_->now();
      auto end_sec = t.seconds();
      auto start = t - history_dur;
      auto start_sec = start.seconds();

      ImPlot::SetNextPlotLimitsX(start_sec, end_sec, ImGuiCond_Always);

      // TODO configurable limits per-axis
      ImPlot::SetNextPlotLimitsY(-2, 2);

      size_t n_ticks = history_length_;
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
          "plot1", "t (header.stamp sec)", nullptr, ImVec2(-1, -1), ImPlotFlags_None))
      {
        for (auto & [_, subscription] : node_->topics_to_subscriptions) {
          std::unique_lock<std::mutex> lock(subscription.buffers_mutex_);
          for (auto & buffer : subscription.buffers) {
            buffer.clear_data_up_to(start);
            {
              std::unique_lock<std::mutex> lock(buffer.data_mutex);
              ImPlot::PlotLineG(
                buffer.axis_name.c_str(),
                &circular_buffer_access,
                &buffer.data,
                buffer.data.size());
            }
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

  int run()
  {
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit()) {
      return EXIT_FAILURE;
    }

    const char * glsl_version = "#version 150";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glEnable(GL_MULTISAMPLE);

    GLFWwindow * window = glfwCreateWindow(1280, 720, "imgui_vendor example", NULL, NULL);
    if (window == NULL) {
      return EXIT_FAILURE;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // enable vsync

    bool err = glewInit() != GLEW_OK;
    if (err) {
      fprintf(stderr, "Failed to initialize OpenGL loader!\n");
      return EXIT_FAILURE;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO & io = ImGui::GetIO();
    io.ConfigFlags &= ~(ImGuiConfigFlags_ViewportsEnable);
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    ImGui::StyleColorsClassic();
    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    while (rclcpp::ok()) {
      if (glfwWindowShouldClose(window)) {
        // ensure ROS finishes up if window is closed
        rclcpp::shutdown();
        break;
      }
      // Poll and handle events (inputs, window resize, etc.)
      // You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
      // - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application.
      // - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application.
      // Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
      glfwPollEvents();

      ImGui_ImplOpenGL3_NewFrame();
      ImGui_ImplGlfw_NewFrame();
      ImGui::NewFrame();

      auto dock_id = ImGui::DockSpaceOverViewport(ImGui::GetMainViewport());
      ImGui::SetNextWindowDockID(dock_id);
      update();
      ImGui::Render();
      int display_w, display_h;
      glfwGetFramebufferSize(window, &display_w, &display_h);
      glViewport(0, 0, display_w, display_h);
      glClearColor(
        clear_color.x * clear_color.w, clear_color.y * clear_color.w,
        clear_color.z * clear_color.w, clear_color.w);
      glClear(GL_COLOR_BUFFER_BIT);
      ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

      glfwSwapBuffers(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    return EXIT_SUCCESS;
  }
};

} // namespace quickplot

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<quickplot::QuickPlotNode>();
  std::thread ros_thread([ = ] {
      rclcpp::spin(node);
    });
  quickplot::Application app(node);
  auto exit_code = app.run();
  ros_thread.join();
  return exit_code;
}
