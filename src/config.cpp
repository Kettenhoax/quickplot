#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <string>
#include <iostream>
#include <vector>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include "quickplot/config.hpp"

namespace fs = std::filesystem;

namespace YAML
{

template<>
struct convert<quickplot::DataSourceConfig>
{
  static Node encode(const quickplot::DataSourceConfig & s)
  {
    Node node;
    node["topic_name"] = s.topic_name;
    node["member_path"] = s.member_path;
    if (s.axis != 0) {
      node["axis"] = s.axis;
    }
    return node;
  }

  static bool decode(const Node & node, quickplot::DataSourceConfig & s)
  {
    s.topic_name = node["topic_name"].as<std::string>();
    s.member_path = node["member_path"].as<std::vector<std::string>>();
    if (node["axis"].IsDefined()) {
      s.axis = node["axis"].as<int>();
      if (s.axis < 0 || s.axis > 2) {
        return false;
      }
    } else {
      s.axis = 0;
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
    node["sources"] = s.sources;
    return node;
  }

  static bool decode(const Node & node, quickplot::PlotConfig & s)
  {
    s.axes = node["axes"].as<std::vector<quickplot::AxisConfig>>();
    s.sources = node["sources"].as<std::vector<quickplot::DataSourceConfig>>();
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
  // TODO(ZeilingerM) validate topic and member names
  try {
    YAML::Node config = YAML::LoadFile(path.string());
    return config.as<ApplicationConfig>();
  } catch (const YAML::Exception &) {
    std::throw_with_nested(config_error());
  }
}

} // namespace quickplot
