#pragma once
#include <string>
#include <vector>
#include <filesystem>
#include <rclcpp/duration.hpp>

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
  std::vector<std::string> member_path;
  // axis 0, 1 or 2
  int axis;
};

struct AxisConfig
{
  double y_min;
  double y_max;
};

struct PlotConfig
{
  std::vector<DataSourceConfig> sources;
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
