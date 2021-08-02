#include "implot.h" // NOLINT
#include "imgui_impl_glfw.h" // NOLINT
#include "imgui_impl_opengl3.h" // NOLINT
#include <GL/glew.h>
#include <GLFW/glfw3.h>

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
#include <rclcpp/rclcpp.hpp>
#include <rosidl_typesupport_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>
#include "quickplot/message_parser.hpp"
#include "quickplot/config.hpp"
#include <boost/circular_buffer.hpp>

using std::placeholders::_1;

namespace quickplot
{

using PlotDataCircularBuffer = boost::circular_buffer<ImPlotPoint>;

struct PlotDataBuffer
{
  MessageMember member;

  // lock this mutex when accessing data from the plot buffer
  std::mutex data_mutex;
  PlotDataCircularBuffer data;

  PlotDataBuffer(MessageMember _member, size_t capacity)
  : member(_member), data_mutex(), data(capacity)
  {

  }

  void clear_data_up_to(rclcpp::Time t)
  {
    std::unique_lock<std::mutex> lock(data_mutex);
    auto s = t.seconds();
    while (!data.empty()) {
      if (data.front().x < s) {
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
  : deserializer_(deserializer)
  {
    message_buffer_ = deserializer_->init_buffer();
    subscription_ = rclcpp::create_generic_subscription(
      topics_interface,
      topic_name,
      deserializer_->message_type(),
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

  void add_field(MessageMember member, size_t capacity)
  {
    std::unique_lock<std::mutex> lock(buffers_mutex_);
    buffers.emplace_back(
      member,
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
        buffer.data.set_capacity(buffer.data.capacity() * 2);
      }
      buffer.data.push_back(
        ImPlotPoint(
          stamp.seconds(),
          deserializer_->get_numeric(message_buffer_.data(), buffer.member.info)));
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
    std::string topic,
    std::shared_ptr<MessageIntrospection> introspection,
    MessageMember member)
  {
    std::unique_lock<std::mutex> lock(topic_mutex);
    auto message_type = introspection->message_type();
    // ensure message parser is initialized
    if (type_to_parsers_.find(message_type) == type_to_parsers_.end()) {
      type_to_parsers_.emplace(
        message_type,
        std::make_shared<IntrospectionMessageDeserializer>(introspection));
    }
    auto it = topics_to_subscriptions.try_emplace(
      topic,
      topic, get_node_topics_interface(), type_to_parsers_.at(message_type));
    auto & subscription = it.first->second;
    size_t capacity = 10; // TODO(ZeilingerM) compute capacity from expected frequencies
    subscription.add_field(member, capacity);
  }
};

ImPlotPoint circular_buffer_access(void * data, int idx)
{
  auto buffer = reinterpret_cast<PlotDataCircularBuffer *>(data);
  return buffer->at(idx);
}

static void glfw_error_callback(int error, const char * description)
{
  fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

struct MemberPayload
{
  const char * topic_name;
  size_t member_index;
};

class Application
{
private:
  std::shared_ptr<QuickPlotNode> node_;
  rclcpp::Event::SharedPtr graph_event_;

  std::string imgui_ini_path_;
  double history_length_;
  double edited_history_length_;
  std::vector<double> history_tick_values_;
  std::vector<std::string> history_tick_labels_;

  std::unordered_map<std::string, std::string> available_topics_to_types_;
  std::unordered_map<std::string,
    std::shared_ptr<MessageIntrospection>> message_type_to_introspection_;
  std::unordered_map<std::string, ImPlotPoint> active_topics_to_data_;

  // queue of requested topic data in plots, for which the topic has not been received on the
  // graph yet
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
    imgui_ini_path_ = get_default_config_directory() + "/imgui.ini";
  }

  void apply_config(ApplicationConfig config)
  {
    history_length_ = config.history_length;
    edited_history_length_ = history_length_;
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
        auto introspection = message_type_to_introspection_.at(type_it->second);
        for (const auto & plot_member : it->members) {
          MessageMember member;
          auto member_info_opt = introspection->get_member_info(plot_member.path);
          if (member_info_opt.has_value()) {
            member.path = plot_member.path;
            member.info = member_info_opt.value();
            node_->add_topic_field(
              it->topic_name, introspection, member);
          } else {
            // TODO show error and warning
            std::cerr << "Could not find member " << plot_member.path[0] << " in message type " <<
              type_it->second << std::endl;
          }
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
              .path = buffer.member.path
            });
        }
      }
    }
    return config;
  }

  void save_application_config()
  {
    save_config(collect_config(), get_default_config_path());
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
      for (const auto & [topic, type] : available_topics_to_types_) {
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
            ImGui::Text("%s", member_formatted.c_str());
            if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_SourceAllowNullID)) {
              MemberPayload payload {
                .topic_name = topic.c_str(),
                .member_index = i,
              };
              ImGui::SetDragDropPayload("topic_member", &payload, sizeof(payload));
              ImGui::EndDragDropSource();
            }
          }
          ImGui::TreePop();
        }
      }
      ImGui::End();
      if (ImGui::Begin("plot view0")) {
        ImGui::InputDouble(
          "history", &edited_history_length_, 1, 10, "%.1f s",
          ImGuiInputTextFlags_CharsDecimal | ImGuiInputTextFlags_CharsNoBlank);
        if (ImGui::IsItemDeactivatedAfterEdit()) {
          history_length_ = edited_history_length_;
        }
        auto history_dur = rclcpp::Duration::from_seconds(history_length_);
        auto t = node_->now();
        auto end_sec = t.seconds();
        auto start = t - history_dur;
        auto start_sec = start.seconds();

        ImPlot::SetNextPlotLimitsX(start_sec, end_sec, ImGuiCond_Always);
        // TODO configurable limits per-axis
        ImPlot::SetNextPlotLimitsY(-2, 2, ImGuiCond_Once);

        size_t n_ticks = static_cast<size_t>(history_length_);
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

        std::vector<char const *> tick_cstrings;
        tick_cstrings.reserve(history_tick_labels_.size());
        for (const auto & labels : history_tick_labels_) {
          tick_cstrings.push_back(const_cast<char *>(labels.c_str()));
        }
        ImPlot::SetNextPlotTicksX(
          history_tick_values_.data(),
          history_tick_values_.size(), tick_cstrings.data());

        if (ImPlot::BeginPlot(
            "plot0", "t (header.stamp sec)", nullptr, ImVec2(-1, -1), ImPlotFlags_None))
        {
          for (auto & [topic_name, subscription] : node_->topics_to_subscriptions) {
            std::unique_lock<std::mutex> lock(subscription.buffers_mutex_);
            for (auto & buffer : subscription.buffers) {
              buffer.clear_data_up_to(start);
              std::stringstream ss;
              ss << topic_name << "/" << boost::algorithm::join(
                buffer.member.path, ".");
              auto axis_name = ss.str();
              {
                std::unique_lock<std::mutex> lock(buffer.data_mutex);
                ImPlot::PlotLineG(
                  axis_name.c_str(),
                  &circular_buffer_access,
                  &buffer.data,
                  buffer.data.size());
              }
            }
          }
          if (ImPlot::BeginDragDropTarget()) {
            if (const ImGuiPayload * payload = ImGui::AcceptDragDropPayload("topic_name")) {
              auto topic_name = std::string(static_cast<char *>(payload->Data));
              auto message_type = available_topics_to_types_.at(topic_name);
              auto introspection = message_type_to_introspection_.at(message_type);
              // TODO don't take first, but recommended default member
              auto first_member_info = *introspection->begin_member_infos();
              node_->add_topic_field(topic_name, introspection, first_member_info);
            }
            ImPlot::EndDragDropTarget();
          }
          if (ImPlot::BeginDragDropTargetY(ImPlotYAxis_1)) {
            if (const ImGuiPayload * payload = ImGui::AcceptDragDropPayload("topic_member")) {
              auto member_payload = static_cast<MemberPayload *>(payload->Data);
              auto message_type = available_topics_to_types_.at(member_payload->topic_name);
              auto introspection = message_type_to_introspection_.at(message_type);
              auto member_infos = introspection->begin_member_infos();
              std::advance(member_infos, member_payload->member_index);
              node_->add_topic_field(member_payload->topic_name, introspection, *member_infos);
            }
            ImPlot::EndDragDropTarget();
          }
          ImPlot::EndPlot();
        }
      }
      ImGui::End();
    }
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
    glfwWindowHint(GLFW_SAMPLES, 8);

    GLFWwindow * window = glfwCreateWindow(1280, 720, "quickplot", NULL, NULL);
    if (window == NULL) {
      return EXIT_FAILURE;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);   // enable vsync

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
    io.IniFilename = imgui_ini_path_.c_str();

    ImGui::StyleColorsDark();
    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    ImVec4 clear_color = ImGui::GetStyle().Colors[ImGuiCol_WindowBg];

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

    // save before closing
    save_application_config();

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
