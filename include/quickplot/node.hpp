#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <string>
#include <utility>
#include <memory>
#include "quickplot/plot_subscription.hpp"

namespace quickplot
{

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

} // namespace quickplot
