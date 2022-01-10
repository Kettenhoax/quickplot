#pragma once
#include <vector>
#include <utility>
#include <memory>
#include <string>
#include "quickplot/config.hpp"
#include "quickplot/introspection.hpp"
#include "quickplot/plot_subscription.hpp"

namespace quickplot
{

// helper for variant visit
template<class ... Ts>
struct overloaded : Ts ... { using Ts::operator() ...; };
template<class ... Ts>
overloaded(Ts ...)->overloaded<Ts...>;

struct MemberPayload
{
  std::string topic_name;
  MemberSequencePath member;
};

struct SeriesPayload
{
  MemberPayload member;
  int axis;
};

enum class DataSourceError
{
  None = 0,
  // the message type behind the data source is not available on the system running the plot tool
  MessageTypeUnavailable,
  // the data source requests a member that is not part of the message type
  InvalidMember,
};

enum class DataWarning
{
  None = 0,
  // the timestamps in the last received messages were out of range
  TimeStampOutOfRange = 1,
};

struct SourceDescriptor
{
  std::string resolved_topic_name;
  MemberSequencePathDescriptor member_path;
  DataSourceError error;
};

struct ActiveDataSource
{
  DataWarning warning;

  // pointer to subscription which fills the buffer
  std::shared_ptr<PlotSubscription> subscription;

  // resolved member path of topic type
  MemberSequencePath member;

  // buffer to time series
  std::shared_ptr<PlotDataBuffer> data;
};

// data sources may be uninitialized, or have failed to do so due to runtime error, in which case
// they are managed as descriptors to display errors to the user
// if they are initialized and active, they manage the data source and buffers
using DataSource = std::variant<SourceDescriptor, ActiveDataSource>;

struct TimeSeries
{
  std::string id;
  DataSource source;
  DataSource stddev_source;

  std::string topic_name() const
  {
    return std::visit(
      overloaded {
        [this](const ActiveDataSource & active) {
          return active.subscription->topic_name();
        },
        [this](const SourceDescriptor & descriptor) {
          return descriptor.resolved_topic_name;
        }
      }, source);
  }

  inline bool operator==(const TimeSeries & other) const
  {
    return id == other.id;
  }
};

struct Plot
{
  // time series and its target plot y axis
  std::vector<std::pair<TimeSeries, int>> series;
  std::vector<AxisConfig> axes;
};

} // namespace quickplot

namespace std
{
template<>
struct hash<quickplot::TimeSeries>
{
  inline size_t operator()(const quickplot::TimeSeries & series) const
  {
    return hash<std::string>().operator()(series.topic_name());
  }
};
} // namespace std
