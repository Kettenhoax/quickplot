#pragma once

#include <memory>
#include <string>
#include <map>
#include <utility>
#include <unordered_map>
#include "quickplot/introspection.hpp"
#include "quickplot/plot.hpp"

namespace quickplot
{

struct MessageTypeError
{
  std::string message_type;
  std::string error_message;
};

using MessageIntrospectionPtr = std::shared_ptr<MessageIntrospection>;

// either a loaded and initialized introspection service, information on why it isn't available
using MessageTypeInfo = std::variant<MessageIntrospectionPtr, MessageTypeError>;

DataSourceConfig source_to_config(const ActiveDataSource & source)
{
  DataSourceConfig config;
  config.topic_name = source.subscription->topic_name();
  config.member_path = to_descriptor(source.accessor.member);
  config.op = source.accessor.op;
  return config;
}

TimeSeriesConfig series_to_config(const std::pair<TimeSeries, int> & p)
{
  TimeSeriesConfig config;
  const auto & [series, axis] = p;
  auto active = std::get_if<ActiveDataSource>(&series.source);
  if (active) {
    config.source = source_to_config(*active);
  }
  auto stddev_active = std::get_if<ActiveDataSource>(&series.stddev_source);
  if (stddev_active) {
    config.stddev_source = source_to_config(*stddev_active);
  }
  config.axis = axis;
  return config;
}

PlotConfig plot_to_config(const Plot & plot)
{
  PlotConfig config;
  std::transform(
    plot.series.begin(), plot.series.end(), std::inserter(config.series, config.series.begin()),
    &series_to_config);
  config.axes = plot.axes;
  return config;
}

// construct id of a time series based on an unresolved member path
std::string series_id(const DataSourceConfig & source_config)
{
  std::stringstream ss;
  ss << source_config.topic_name << "/" << source_config.member_path;
  if (source_config.op == DataSourceOperator::Sqrt) {
    ss << "-sqrt";
  }
  return ss.str();
}

// construct id of a time series based on a resolved member path
// should match the id based on the unresolved path, to correctly identify a series on the GUI
// across the initialization process
std::string series_id(const std::string & topic, const MessageAccessor & accessor)
{
  std::stringstream ss;
  ss << topic << "/" << accessor.member;
  if (accessor.op == DataSourceOperator::Sqrt) {
    ss << "-sqrt";
  }
  return ss.str();
}

const char * get_message_type(const MessageTypeInfo & type_info)
{
  return std::visit(
    overloaded {
      [](const std::shared_ptr<MessageIntrospection> & introspection) {
        return introspection->message_type();
      },
      [](const MessageTypeError & error) {
        return error.message_type.c_str();
      }
    }, type_info);
}

class IntrospectionCache
{
private:
  std::unordered_map<std::string, MessageIntrospectionPtr> cache_;

public:
  IntrospectionCache() = default;

  MessageIntrospectionPtr load(const std::string & message_type)
  {
    auto it = cache_.find(message_type);
    if (it != cache_.end()) {
      return it->second;
    }
    const auto & [new_entry, _] = cache_.emplace(
      message_type,
      std::make_shared<MessageIntrospection>(message_type));
    return new_entry->second;
  }
};

using TopicTypeMap = std::map<std::string, MessageTypeInfo>;

} // namespace quickplot
