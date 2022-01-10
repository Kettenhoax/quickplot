#include <gmock/gmock.h>
#include <limits>
#include <string>
#include "quickplot/config.hpp"
#include <filesystem>

namespace fs = std::filesystem;
using ::testing::StrEq;

TEST(test_config, parse_example_config) {
  auto config = quickplot::load_config("test/example_config.yaml");
  EXPECT_NEAR(config.history_length, 5.0, std::numeric_limits<double>::epsilon());
  EXPECT_EQ(config.plots.size(), 1lu);

  auto plot = config.plots[0];
  EXPECT_EQ(plot.axes.size(), 1u);
  EXPECT_EQ(plot.axes[0].y_min, -2.0);
  EXPECT_EQ(plot.axes[0].y_max, 2.0);
  EXPECT_EQ(plot.series.size(), 2lu);

  // TODO(ZeilingerM): order of items in set is not defined
  // need a way to test set for exactly two specific struct items
  auto it = plot.series.begin();
  auto source = it->source;
  EXPECT_EQ(source.topic_name, "sequence");
  ASSERT_EQ(source.member_path.size(), 2lu);
  EXPECT_THAT(source.member_path[0].member_name, StrEq("inner"));
  EXPECT_EQ(source.member_path[0].sequence_idx, 1lu);
  EXPECT_THAT(source.member_path[1].member_name, StrEq("value"));
  EXPECT_FALSE(source.member_path[1].sequence_idx.has_value());

  ASSERT_TRUE(it->stddev_source.has_value());
  source = it->stddev_source.value();
  EXPECT_EQ(source.topic_name, "sequence");
  ASSERT_EQ(source.member_path.size(), 1lu);
  EXPECT_THAT(source.member_path[0].member_name, StrEq("field"));
  EXPECT_FALSE(source.member_path[0].sequence_idx.has_value());

  ++it;
  source = it->source;
  EXPECT_EQ(source.topic_name, "/flat");
  ASSERT_EQ(source.member_path.size(), 2lu);
  EXPECT_THAT(source.member_path[0].member_name, StrEq("inner"));
  EXPECT_FALSE(source.member_path[0].sequence_idx.has_value());
  EXPECT_THAT(source.member_path[1].member_name, StrEq("value"));
  EXPECT_FALSE(source.member_path[1].sequence_idx.has_value());

  EXPECT_FALSE(it->stddev_source.has_value());
}
