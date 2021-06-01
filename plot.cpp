#include <Mahi/Gui.hpp>
#include <mutex>
#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <four_wheel_steering_msgs/msg/four_wheel_steering_stamped.hpp>

using namespace mahi::gui;
using namespace mahi::util;
using four_wheel_steering_msgs::msg::FourWheelSteeringStamped;
using std::placeholders::_1;

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
    subscription_ = this->create_subscription<FourWheelSteeringStamped>(
      "cmd_4ws", 10, std::bind(&BufferNode::data_callback, this, _1));
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
  void data_callback(const FourWheelSteeringStamped::SharedPtr msg)
  {
    TopicData data;
    data.t = rclcpp::Time(msg->header.stamp);
    data.value = msg->data.speed;
    {
      std::unique_lock<std::mutex> lock(data_mutex_);
      data_queue_.push_back(data);
    }
  }

  rclcpp::Subscription<FourWheelSteeringStamped>::SharedPtr subscription_;
};

class QuickPlot : public Application
{
private:
  std::shared_ptr<BufferNode> node_;
  std::vector<double> times_;
  std::vector<double> data_;

  std::vector<double> history_tick_values_;
  std::vector<std::string> history_tick_labels_;

public:
  QuickPlot(std::shared_ptr<BufferNode> _node)
  : Application(640, 480, "QuickPlot"), node_(_node) {
    set_vsync(false);
    ImGui::DisableViewports();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  }

  void update() override
  {
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
    auto vec2 = get_window_size();
    ImGui::SetNextWindowPos({0,0}, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(vec2.x, vec2.y), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowBgAlpha(0.0f);

    if (!ImGui::Begin("QuickPlot", nullptr, ImGuiWindowFlags_NoTitleBar)) {
      // Early out if the window is collapsed, as an optimization.
      ImGui::End();
      return;
    }
    ImGui::PopStyleVar();

    static ImGuiDockNodeFlags dockspace_flags = ImGuiDockNodeFlags_PassthruCentralNode | ImGuiWindowFlags_NoBackground;
    ImGuiID dockspace_id = ImGui::DockSpaceOverViewport(ImGui::GetMainViewport());

    ImGui::SetNextWindowDockID(dockspace_id, ImGuiCond_FirstUseEver);

    node_->take(times_, data_);

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
      ImPlot::PlotLine(
        "cmd_4ws", times_.data(), data_.data(), times_.size());
      ImPlot::EndPlot();
    }
    ImGui::End();
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BufferNode>();
  std::thread ros_thread([ = ] {
      rclcpp::spin(node);
    });
  QuickPlot app(node);
  app.run();
  return EXIT_SUCCESS;
}
