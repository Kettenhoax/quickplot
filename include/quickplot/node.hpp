#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <map>
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
  std::map<std::string, PlotSubscription> topics_to_subscriptions;

  void add_topic_field(
    std::string topic,
    std::shared_ptr<MessageIntrospection> introspection,
    MemberPath member)
  {
    std::unique_lock<std::mutex> lock(topic_mutex);
    auto it = topics_to_subscriptions.try_emplace(
      topic,
      topic, get_node_topics_interface(), get_node_clock_interface(),
      std::make_shared<IntrospectionMessageDeserializer>(introspection));
    auto & subscription = it.first->second;
    subscription.add_field(member, 1);
  }
};

} // namespace quickplot
