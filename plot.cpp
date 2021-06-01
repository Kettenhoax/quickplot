#include <Mahi/Gui.hpp>
#include <Mahi/Util.hpp>

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

public:
  QuickPlot(std::shared_ptr<BufferNode> _node)
  : Application(640, 480, "QuickPlot"), node_(_node) {}

  void update() override
  {
    ImGui::Begin("QuickPlot");
    node_->take(times_, data_);

    static float history_sec = 10.0f;
    ImGui::SliderFloat("history", &history_sec, 1, 30, "%.1f s");
    auto history_dur = rclcpp::Duration::from_seconds(history_sec);
    auto t = node_->now();
    auto end_sec = t.seconds();
    auto start_sec = (t - history_dur).seconds();

    static ImPlotAxisFlags flags = ImPlotAxisFlags_NoTickLabels;
    ImPlot::SetNextPlotLimitsX(start_sec, end_sec, ImGuiCond_Always);
    ImPlot::SetNextPlotLimitsY(-2, 2);
    if (ImPlot::BeginPlot(
        "speed", "t", "m/s", ImGui::ImVec2(-1, -1), ImPlot::ImPlotFlags_None,
        ImPlot::ImPlotAxisFlags_Lock))
    {
      int n_ticks = history_sec;
      std::vector<char*> 
      ImPlot::SetNextPlotTicksX()
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
