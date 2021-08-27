#include <gmock/gmock.h>
#include <limits>
#include "quickplot/config.hpp"
#include <filesystem>
namespace fs = std::filesystem;

TEST(test_config, parse_example_config) {
  auto config = quickplot::load_config("test/example_config.yaml");
  EXPECT_NEAR(config.history_length, 5.0, std::numeric_limits<double>::epsilon());
  EXPECT_EQ(config.plots.size(), 1lu);

  auto plot = config.plots[0];
  EXPECT_EQ(plot.axes.size(), 1u);
  EXPECT_EQ(plot.axes[0].y_min, -2.0);
  EXPECT_EQ(plot.axes[0].y_max, 2.0);
  EXPECT_EQ(plot.sources.size(), 1lu);
  EXPECT_EQ(plot.sources.begin()->topic_name, "test");
  ASSERT_THAT(plot.sources.begin()->member_path, ::testing::ElementsAre("inner", "value"));
}
