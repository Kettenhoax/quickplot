#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <string>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include "quickplot/config.hpp"

namespace YAML
{
template<>
struct convert<quickplot::MessageMemberPlotConfig>
{
  static Node encode(const quickplot::MessageMemberPlotConfig & mmpc)
  {
    Node node;
    node["path"] = mmpc.path;
    return node;
  }

  static bool decode(const Node & node, quickplot::MessageMemberPlotConfig & mmpc)
  {
    auto path_node = node["path"];
    if (!path_node.IsSequence()) {
      return false;
    }
    for (const auto & item : path_node) {
      mmpc.path.push_back(item.as<std::string>());
    }
    return true;
  }
};

template<>
struct convert<quickplot::TopicPlotConfig>
{
  static Node encode(const quickplot::TopicPlotConfig & topic_plot)
  {
    Node node;
    node["topic_name"] = topic_plot.topic_name;
    if (!topic_plot.members.empty()) {
      auto members = node["members"];
      for (const auto & member : topic_plot.members) {
        members.push_back(member);
      }
    }
    return node;
  }

  static bool decode(const Node & node, quickplot::TopicPlotConfig & tpc)
  {
    auto members_node = node["members"];
    if (!members_node.IsSequence()) {
      return false;
    }
    for (const auto & item : members_node) {
      tpc.members.push_back(item.as<quickplot::MessageMemberPlotConfig>());
    }
    tpc.topic_name = node["topic_name"].as<std::string>();
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
    if (!config.topic_plots.empty()) {
      auto plots = node["topic_plots"];
      for (const auto & topic_plot : config.topic_plots) {
        plots.push_back(topic_plot);
      }
    }
    return node;
  }

  static bool decode(const Node & node, quickplot::ApplicationConfig & config)
  {
    auto topics_node = node["topic_plots"];
    if (topics_node.IsDefined()) {
      if (!topics_node.IsSequence()) {
        return false;
      }
      for (const auto & item : topics_node) {
        config.topic_plots.push_back(item.as<quickplot::TopicPlotConfig>());
      }
    }
    config.history_length = node["history_length"].as<double>();
    return true;
  }
};
} // namespace YAML

namespace quickplot
{

std::string get_home_directory()
{
  // https://stackoverflow.com/questions/2910377/get-home-directory-in-linux
  const char * homedir;
  if ((homedir = getenv("HOME")) == NULL) {
    homedir = getpwuid(getuid())->pw_dir; // NOLINT (only required once on launch)
  }
  return homedir;
}

std::string get_default_config_path()
{
  return get_home_directory() + "/.quickplot/default.yaml";
}

void save_config(ApplicationConfig config, std::string path)
{
  std::ofstream fout;
  fout.open(path);
  if (!fout.is_open()) {
    throw std::invalid_argument("cannot save to path");
  }
  YAML::Node node = YAML::convert<ApplicationConfig>::encode(config);
  fout << node;
  fout << std::endl;
  fout.close();
}

ApplicationConfig default_config()
{
  return ApplicationConfig {
    .history_length = 10.0,
    .topic_plots = {}
  };
}

ApplicationConfig load_config(std::string path)
{
  // TODO validate topic and member names
  YAML::Node config = YAML::LoadFile(path);
  return config.as<ApplicationConfig>();
}

} // namespace quickplot
