#pragma once
#include <string>
#include <vector>
#include <unordered_set>
#include <filesystem>
#include <rclcpp/duration.hpp>
#include <quickplot/introspection.hpp>

namespace fs = std::filesystem;

namespace quickplot
{

struct config_error : public std::exception
{
  const char * what() const throw ()
  {
    return "error loading application configuration";
  }
};

struct DataSourceConfig
{
  std::string topic_name;
  MemberSequencePathDescriptor member_path;
  // axis 0, 1 or 2
  int axis;

  inline bool operator==(const DataSourceConfig & other) const
  {
    return topic_name == other.topic_name && member_path == other.member_path && axis == other.axis;
  }
};

} // namespace quickplot

namespace std
{
template<>
struct hash<quickplot::DataSourceConfig>
{
  inline size_t operator()(const quickplot::DataSourceConfig & config) const
  {
    return hash<std::string>().operator()(config.topic_name);
  }
};
} // namespace std

namespace quickplot
{

struct AxisConfig
{
  double y_min;
  double y_max;
};

struct PlotConfig
{
  std::unordered_set<DataSourceConfig> sources;
  std::vector<AxisConfig> axes;
};

struct ApplicationConfig
{
  double history_length;
  std::vector<PlotConfig> plots;
};

fs::path get_default_config_directory();

fs::path get_default_config_path();

ApplicationConfig default_config();

void save_config(ApplicationConfig, fs::path);

ApplicationConfig load_config(fs::path);

} // namespace quickplot
