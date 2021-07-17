#pragma once
#include <string>
#include <vector>
#include <rclcpp/duration.hpp>

namespace quickplot
{

struct MessageMemberPlotConfig
{
  std::vector<std::string> path;
};

struct TopicPlotConfig
{
  std::string topic_name;
  std::vector<MessageMemberPlotConfig> members;
};

struct ApplicationConfig
{
  double history_length;
  std::vector<TopicPlotConfig> topic_plots;
};

std::string get_default_config_path();

ApplicationConfig default_config();

void save_config(ApplicationConfig, std::string);

ApplicationConfig load_config(std::string);

} // namespace quickplot
