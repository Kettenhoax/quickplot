#pragma once

#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <map>
#include <string>
#include <utility>
#include <list>
#include <memory>
#include "quickplot/plot_subscription.hpp"

namespace quickplot
{

class QuickPlotNode : public rclcpp::Node
{
private:
  std::mutex topic_mutex_;
  std::list<std::weak_ptr<PlotSubscription>> subscriptions_;

public:
  QuickPlotNode() : Node("quickplot")
  {
  }

  std::shared_ptr<PlotSubscription> get_or_create_subscription(
    std::string topic,
    std::shared_ptr<MessageIntrospection> introspection)
  {
    std::unique_lock<std::mutex> lock(topic_mutex_);

    auto it = subscriptions_.begin();
    while (it != subscriptions_.end()) {
      auto subscription = it->lock();
      if (subscription) {
        if (subscription->topic_name() == topic) {
          return subscription;
        }
        ++it;
      } else {
        it = subscriptions_.erase(it);
      }
    }

    auto new_subscription = std::make_shared<PlotSubscription>(
      topic, get_node_topics_interface(), get_node_clock_interface(),
      std::make_shared<IntrospectionMessageDeserializer>(introspection));
    subscriptions_.emplace_back(new_subscription);
    return new_subscription;
  }

  bool is_subscribed_to(std::string topic)
  {
    std::unique_lock<std::mutex> lock(topic_mutex_);
    auto it = subscriptions_.begin();
    while (it != subscriptions_.end()) {
      auto subscription = it->lock();
      if (subscription) {
        if (subscription->topic_name() == topic) {
          return true;
        }
        ++it;
      } else {
        it = subscriptions_.erase(it);
      }
    }
    return false;
  }

  void clear()
  {
    std::unique_lock<std::mutex> lock(topic_mutex_);
    auto it = subscriptions_.begin();
    while (it != subscriptions_.end()) {
      auto subscription = it->lock();
      if (subscription) {
        subscription->clear();
        ++it;
      } else {
        it = subscriptions_.erase(it);
      }
    }
  }
};

} // namespace quickplot
