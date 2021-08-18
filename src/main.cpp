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

namespace fs = std::filesystem;
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
  std::mutex buffers_mutex_;

  // one data buffer per plotted member of a message
  // using list instead of vector, since the emplace_back operation does not require
  // the element to be MoveConstructible for resizing the array
  std::list<PlotDataBuffer> buffers;

public:
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
    for (auto & buffer : buffers) {
      if (buffer.member.path == member.path) {
        std::invalid_argument("member already in subscription");
      }
    }
    buffers.emplace_back(
      member,
      capacity
    );
  }

  PlotDataBuffer & get_buffer(std::vector<std::string> member_path)
  {
    std::unique_lock<std::mutex> lock(buffers_mutex_);
    for (auto & buffer : buffers) {
      if (buffer.member.path == member_path) {
        return buffer;
      }
    }
    throw std::invalid_argument("member not found by path");
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

  std::string imgui_ini_path_;
  double edited_history_length_;
  std::vector<double> history_tick_values_;
  std::vector<std::string> history_tick_labels_;

  std::unordered_map<std::string, std::string> available_topics_to_types_;
  std::unordered_map<std::string,
    std::shared_ptr<MessageIntrospection>> message_type_to_introspection_;
  std::unordered_map<std::string, ImPlotPoint> active_topics_to_data_;

  // queue of requested topic data, for which the topic metadata has not been received yet
  std::mutex data_source_queue_mutex_;
  std::deque<DataSourceConfig> uninitialized_data_source_queue_;

public:
  explicit Application(fs::path config_file, std::shared_ptr<QuickPlotNode> _node)
  : active_config_file_(config_file), node_(_node)
  {
    graph_event_ = node_->get_graph_event();
    graph_event_->set(); // set manually to trigger initial topics query

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
    imgui_ini_path_ = get_default_config_directory().append("imgui.ini").string();
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

  void plot_view(size_t i, PlotConfig & plot, rclcpp::Time t)
  {
    auto id = "plot" + std::to_string(i);
    if (ImGui::Begin(id.c_str())) {
      for (ImPlotYAxis a = 0; a < static_cast<ImPlotYAxis>(plot.axes.size()); a++) {
        const auto & axis_config = plot.axes[a];
        ImPlot::SetNextPlotLimitsY(
          axis_config.y_min, axis_config.y_max,
          ImGuiCond_Once, a);
      }

      if (ImPlot::BeginPlot(
          id.c_str(), "t (header.stamp sec)", nullptr, ImVec2(-1, -1),
          (plot.axes.size() >= 2 ? ImPlotFlags_YAxis2 : 0) |
          (plot.axes.size() >= 3 ? ImPlotFlags_YAxis3 : 0) | ImPlotFlags_NoTitle))
      {
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
                buffer.clear_data_up_to(t);
                series_label = ss.str();
                {
                  std::unique_lock<std::mutex> lock(buffer.data_mutex);
                  ImPlot::PlotLineG(
                    series_label.c_str(),
                    &circular_buffer_access,
                    &buffer.data,
                    buffer.data.size());
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

      auto history_dur = rclcpp::Duration::from_seconds(config_.history_length);
      auto t = node_->now();
      auto end_sec = t.seconds();
      auto start = t - history_dur;
      auto start_sec = start.seconds();

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

      std::vector<char const *> tick_cstrings;
      tick_cstrings.reserve(history_tick_labels_.size());
      for (const auto & labels : history_tick_labels_) {
        tick_cstrings.push_back(const_cast<char *>(labels.c_str()));
      }

      for (size_t i = 0; i < config_.plots.size(); i++) {
        auto & plot = config_.plots[i];
        ImPlot::SetNextPlotLimitsX(start_sec, end_sec, ImGuiCond_Always);
        ImPlot::SetNextPlotTicksX(
          history_tick_values_.data(),
          history_tick_values_.size(), tick_cstrings.data());

        plot_view(i, plot, start);
      }
    }
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
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    ImVec4 clear_color = ImGui::GetStyle().Colors[ImGuiCol_WindowBg];

    while (rclcpp::ok()) {
      if (glfwWindowShouldClose(window)) {
        // ensure ROS finishes up if window is closed
        rclcpp::shutdown();
        break;
      }
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

    // save configuration before closing
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
  auto non_ros_args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  auto node = std::make_shared<quickplot::QuickPlotNode>();
  std::thread ros_thread([ = ] {
      rclcpp::spin(node);
    });

  fs::path config_file;
  if (non_ros_args.size() > 1) {
    // non ROS arguments after the program name are interpreted as config file paths
    config_file = non_ros_args[1];
  } else {
    config_file = quickplot::get_default_config_path();
  }
  quickplot::Application app(config_file, node);
  auto exit_code = app.run();
  ros_thread.join();
  return exit_code;
}
