// include implot.h for ImPlotPoint struct, to avoid copies when plotting
#include "implot.h" // NOLINT

#include "quickplot/message_parser.hpp"
#include <libstatistics_collector/moving_average_statistics/moving_average.hpp>
#include <mutex>
#include <string>
#include <utility>
#include <memory>
#include <deque>
#include <list>
#include <vector>
#include <boost/circular_buffer.hpp>

namespace quickplot
{
using std::placeholders::_1;
using PlotDataCircularBuffer = boost::circular_buffer<ImPlotPoint>;
using libstatistics_collector::moving_average_statistics::MovingAverageStatistics;
using libstatistics_collector::moving_average_statistics::StatisticData;

struct PlotDataBuffer
{
  MessageMember member;

  // lock this mutex when accessing data from the plot buffer
  mutable std::mutex data_mutex;
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
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface_;
  rclcpp::GenericSubscription::SharedPtr subscription_;

  rclcpp::Time last_received_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  MovingAverageStatistics receive_period_stats_;

  // protect access to list of buffers
  mutable std::mutex buffers_mutex_;

  // One data buffer per plotted member of a message.
  //
  // Using list instead of vector, since the emplace_back operation does not require
  // the element to be MoveConstructible for resizing the array. The data buffer should
  // not be move constructed.
  std::list<PlotDataBuffer> buffers;

public:
  explicit PlotSubscription(
    std::string topic_name,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface,
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock_interface,
    std::shared_ptr<IntrospectionMessageDeserializer> deserializer)
  : deserializer_(deserializer), node_clock_interface_(clock_interface)
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

  StatisticData receive_period_stats() const
  {
    return receive_period_stats_.GetStatistics();
  }

  void receive_callback(std::shared_ptr<rclcpp::SerializedMessage> message)
  {
    auto t_steady = steady_clock_.now();
    if (last_received_.nanoseconds() != 0) {
      receive_period_stats_.AddMeasurement((t_steady - last_received_).seconds());
    }
    last_received_ = t_steady;

    deserializer_->deserialize(*message, message_buffer_.data());
    auto stamp = deserializer_->get_header_stamp(message_buffer_.data());
    rclcpp::Time t;
    if (stamp.has_value()) {
      t = stamp.value();
    } else {
      t = node_clock_interface_->get_clock()->now();
    }
    std::unique_lock<std::mutex> lock(buffers_mutex_);
    for (auto & buffer : buffers) {
      std::unique_lock<std::mutex> lock(buffer.data_mutex);
      if (buffer.data.full()) {
        buffer.data.set_capacity(buffer.data.capacity() * 2);
      }
      buffer.data.push_back(
        ImPlotPoint(
          t.seconds(),
          deserializer_->get_numeric(message_buffer_.data(), buffer.member.info)));
    }
  }

  void clear_all_data()
  {
    std::unique_lock<std::mutex> lock(buffers_mutex_);
    for (auto & buffer : buffers) {
      std::unique_lock<std::mutex> lock(buffer.data_mutex);
      buffer.data.clear();
    }
  }
};

} // namespace quickplot
