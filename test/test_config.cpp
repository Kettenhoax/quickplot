#include <gtest/gtest.h>
#include <limits>
#include "quickplot/config.hpp"
#include <filesystem>
namespace fs = std::filesystem;

TEST(test_config, parse_example_config) {
  auto config = quickplot::load_config("test/example_config.yaml");
  EXPECT_NEAR(config.history_length, 5.0, std::numeric_limits<double>::epsilon());
  EXPECT_EQ(config.topic_plots.size(), 1lu);

  EXPECT_EQ(config.topic_plots[0].topic_name, "test");
  EXPECT_EQ(config.topic_plots[0].members.size(), 1lu);
  EXPECT_EQ(config.topic_plots[0].members[0].path.size(), 2lu);
  EXPECT_EQ(config.topic_plots[0].members[0].path[0], "inner");
  EXPECT_EQ(config.topic_plots[0].members[0].path[1], "value");
}

TEST(test_config, write_config) {
  std::FILE * tmpf = std::tmpfile();
  auto path = fs::read_symlink(fs::path("/proc/self/fd") / std::to_string(fileno(tmpf)));

  quickplot::ApplicationConfig config;
  config.history_length = 15.0;
  auto& topic_plot = config.topic_plots.emplace_back();
  topic_plot.topic_name = "test";
  quickplot::save_config(config, path);
}
