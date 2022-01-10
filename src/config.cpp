#include <pwd.h> // for accessing home directory
#include <unordered_set>
#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <charconv>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include "quickplot/config.hpp"
#include "quickplot/introspection.hpp"

namespace fs = std::filesystem;

namespace YAML
{

template<typename Value>
struct convert<std::unordered_set<Value>>
{
  static YAML::Node encode(const std::unordered_set<Value> & rhs)
  {
    Node node(NodeType::Sequence);
    for (auto it = std::begin(rhs); it != std::end(rhs); ++it) {
      node.push_back(*it);
    }
    return node;
  }

  static bool decode(const Node & node, std::unordered_set<Value> & rhs)
  {
    if (!node.IsSequence()) {
      return false;
    }
    for (auto it = node.begin(); it != node.end(); ++it) {
      rhs.insert(it->as<Value>());
    }
    return true;
  }
};

template<>
struct convert<quickplot::MemberSequencePathItemDescriptor>
{
  static Node encode(const quickplot::MemberSequencePathItemDescriptor & item)
  {
    Node node(NodeType::Scalar);
    std::stringstream ss;
    write_member_sequence_path_item_descriptor(ss, item);
    node = ss.str();
    return node;
  }

  static bool decode(const Node & node, quickplot::MemberSequencePathItemDescriptor & item)
  {
    auto in_str = node.as<std::string>();
    auto first_bracket = in_str.find_first_of('[');
    if (first_bracket == std::string::npos) {
      item.member_name = in_str;
      item.sequence_idx = std::nullopt;
      return true;
    }
    auto last_bracket = in_str.find_first_of(']', first_bracket + 1);
    if (last_bracket == std::string::npos || last_bracket + 1 != in_str.size()) {
      return false;
    }
    size_t idx;
    auto result = std::from_chars(
      in_str.data() + first_bracket + 1,
      in_str.data() + last_bracket, idx);
    if (result.ec == std::errc::invalid_argument) {
      return false;
    }
    item.member_name = in_str.substr(0, first_bracket);
    item.sequence_idx = idx;
    return true;
  }
};

template<>
struct convert<quickplot::DataSourceConfig>
{
  static Node encode(const quickplot::DataSourceConfig & config)
  {
    Node node;
    node["topic_name"] = config.topic_name;
    node["member_path"] = config.member_path;
    if (config.op == quickplot::DataSourceOperator::Sqrt) {
      node["op"] = "sqrt";
    }
    return node;
  }

  static bool decode(const Node & node, quickplot::DataSourceConfig & config)
  {
    config.topic_name = node["topic_name"].as<std::string>();
    config.member_path = node["member_path"].as<quickplot::MemberSequencePathDescriptor>();
    config.op = quickplot::DataSourceOperator::Identity;
    if (node["op"].IsDefined()) {
      if (node["op"].as<std::string>() == "sqrt") {
        config.op = quickplot::DataSourceOperator::Sqrt;
      }
    }
    return true;
  }
};

template<>
struct convert<quickplot::TimeSeriesConfig>
{
  static Node encode(const quickplot::TimeSeriesConfig & config)
  {
    Node node;
    node["source"] = config.source;
    if (config.stddev_source.has_value()) {
      node["stddev_source"] = config.stddev_source.value();
    }
    if (config.axis != 0) {
      node["axis"] = config.axis;
    }
    return node;
  }

  static bool decode(const Node & node, quickplot::TimeSeriesConfig & config)
  {
    config.source = node["source"].as<quickplot::DataSourceConfig>();
    if (node["stddev_source"].IsDefined()) {
      config.stddev_source = node["stddev_source"].as<quickplot::DataSourceConfig>();
    }
    if (node["axis"].IsDefined()) {
      config.axis = node["axis"].as<int>();
      if (config.axis < 0 || config.axis > 2) {
        return false;
      }
    } else {
      config.axis = 0;
    }
    return true;
  }
};

template<>
struct convert<quickplot::AxisConfig>
{
  static Node encode(const quickplot::AxisConfig & s)
  {
    Node node;
    node["y_min"] = s.y_min;
    node["y_max"] = s.y_max;
    return node;
  }

  static bool decode(const Node & node, quickplot::AxisConfig & s)
  {
    s.y_min = node["y_min"].as<double>();
    s.y_max = node["y_max"].as<double>();
    return true;
  }
};

template<>
struct convert<quickplot::PlotConfig>
{
  static Node encode(const quickplot::PlotConfig & s)
  {
    Node node;
    node["axes"] = s.axes;
    node["series"] = s.series;
    return node;
  }

  static bool decode(const Node & node, quickplot::PlotConfig & s)
  {
    s.axes = node["axes"].as<std::vector<quickplot::AxisConfig>>();
    s.series = node["series"].as<std::unordered_set<quickplot::TimeSeriesConfig>>();
    return true;
  }
};

template<>
struct convert<quickplot::ApplicationConfig>
{
  static Node encode(const quickplot::ApplicationConfig & config)
  {
    Node node;
    node["history_length"] = config.history_length;
    node["plots"] = config.plots;
    return node;
  }

  static bool decode(const Node & node, quickplot::ApplicationConfig & config)
  {
    config.history_length = node["history_length"].as<double>();
    config.plots = node["plots"].as<std::vector<quickplot::PlotConfig>>();
    return true;
  }
};
} // namespace YAML

namespace quickplot
{

constexpr const char * APPLICATION_NAME = "quickplot";

fs::path get_home_directory()
{
  // https://stackoverflow.com/questions/2910377/get-home-directory-in-linux
  const char * homedir;
  if ((homedir = getenv("HOME")) == NULL) {
    homedir = getpwuid(getuid())->pw_dir; // NOLINT (only required once on launch)
  }
  return homedir;
}

fs::path get_default_config_directory()
{
  const char * xdg_config_home;
  if ((xdg_config_home = getenv("XDG_CONFIG_HOME")) == NULL) {
    return get_home_directory().append(".config").append(APPLICATION_NAME);
  }
  return fs::path(xdg_config_home).append(APPLICATION_NAME);
}

fs::path get_default_config_path()
{
  return get_default_config_directory().append("default.yaml");
}

void save_config(ApplicationConfig config, fs::path path)
{
  std::ofstream fout;
  std::ios_base::iostate errors = fout.exceptions() | std::ios::failbit;
  fout.exceptions(errors);
  fout.open(path);
  YAML::Node node = YAML::convert<ApplicationConfig>::encode(config);
  fout << node;
  fout << std::endl;
  fout.close();
}

ApplicationConfig default_config()
{
  return ApplicationConfig {
    .history_length = 10.0,
    .plots = {}
  };
}

ApplicationConfig load_config(fs::path path)
{
  try {
    YAML::Node config = YAML::LoadFile(path.string());
    return config.as<ApplicationConfig>();
  } catch (const YAML::Exception &) {
    std::throw_with_nested(config_error());
  }
}

} // namespace quickplot
