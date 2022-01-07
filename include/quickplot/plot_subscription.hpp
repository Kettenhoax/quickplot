// include implot.h for ImPlotPoint struct, to avoid copies when plotting
#include "implot.h" // NOLINT

#include "quickplot/message_parser.hpp"
#include <libstatistics_collector/moving_average_statistics/moving_average.hpp>
#include <mutex>
#include <string>
#include <utility>
#include <unordered_map>
#include <memory>
#include <deque>
#include <list>
#include <vector>
#include <boost/circular_buffer.hpp>

namespace quickplot
{
using std::placeholders::_1;
using CircularBuffer = boost::circular_buffer<ImPlotPoint>;
using libstatistics_collector::moving_average_statistics::MovingAverageStatistics;
using libstatistics_collector::moving_average_statistics::StatisticData;

class PlotDataBuffer;

/**
 * Immutable random-access-iterator of plot data.
 * This class is made thread-safe by locking the parent container during its entire lifetime.
 */
class PlotDataContainer
{
private:
  const PlotDataBuffer * parent_;
  // optional to allow for empty containers
  std::optional<std::unique_lock<std::mutex>> lock_;

public:
  explicit PlotDataContainer(const PlotDataBuffer * parent);

  PlotDataContainer();

  size_t size() const;

  CircularBuffer::const_iterator begin() const;

  CircularBuffer::const_iterator end() const;
};

// Circular buffer of ImPlotPoint
class PlotDataBuffer
{
  friend class PlotDataContainer;

private:
  // lock this mutex when accessing data from the plot buffer
  mutable std::mutex mutex_;
  CircularBuffer data_;
  std::weak_ptr<PlotDataContainer> active_container_;

public:
  explicit PlotDataBuffer(size_t capacity)
  : mutex_(), data_(capacity)
  {

  }

  std::shared_ptr<PlotDataContainer> data()
  {
    if (!active_container_.expired()) {
      throw std::runtime_error("only one PlotDataContainer can reference this buffer at a time");
    }
    auto container = std::make_shared<PlotDataContainer>(this);
    active_container_ = container;
    return container;
  }

  bool empty() const
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return data_.empty();
  }

  void push(double x, double y)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (data_.full()) {
      data_.set_capacity(data_.capacity() * 2);
    }
    data_.push_back(
      ImPlotPoint(
        x,
        y));
  }

  void clear_data_up_to(rclcpp::Time t)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    auto s = t.seconds();
    while (!data_.empty()) {
      if (data_.front().x < s) {
        data_.pop_front();
      } else {
        break;
      }
    }
  }

  void clear()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    data_.clear();
  }
};

PlotDataContainer::PlotDataContainer(const PlotDataBuffer * parent)
: parent_(parent), lock_(parent_->mutex_)
{

}

PlotDataContainer::PlotDataContainer()
: parent_(nullptr), lock_()
{

}

size_t PlotDataContainer::size() const
{
  if (!parent_) {
    return 0;
  }
  return parent_->data_.size();
}

CircularBuffer::const_iterator PlotDataContainer::begin() const
{
  return parent_->data_.begin();
}

CircularBuffer::const_iterator PlotDataContainer::end() const
{
  return parent_->data_.end();
}

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
  std::list<std::pair<MemberSequencePath, std::weak_ptr<PlotDataBuffer>>> buffers_;

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

  std::string topic_name() const {
    return subscription_->get_topic_name();
  }

  std::shared_ptr<PlotDataBuffer> add_source(MemberSequencePath in_member)
  {
    std::unique_lock<std::mutex> lock(buffers_mutex_);
    for (auto & [member, buffer] : buffers_) {
      if (member == in_member) {
        std::invalid_argument("member already in subscription");
      }
    }
    auto buffer = std::make_shared<PlotDataBuffer>(1);
    buffers_.emplace_back(in_member, buffer);
    return buffer;
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
    {
      std::unique_lock<std::mutex> lock(buffers_mutex_);
      auto it = buffers_.begin();
      while (it != buffers_.end()) {
        auto buffer = it->second.lock();
        if (buffer) {
          double value = get_nested_numeric(message_buffer_.data(), it->first);
          buffer->push(t.seconds(), value);
          ++it;
        } else {
          it = buffers_.erase(it);
        }
      }
    }
  }

  void clear()
  {
    std::unique_lock<std::mutex> lock(buffers_mutex_);
    auto it = buffers_.begin();
    while (it != buffers_.end()) {
      auto buffer = it->second.lock();
      if (buffer) {
        buffer->clear();
        ++it;
      } else {
        it = buffers_.erase(it);
      }
    }
  }
};

} // namespace quickplot
