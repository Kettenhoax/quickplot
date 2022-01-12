#include <gmock/gmock.h>
#include <limits>
#include "quickplot/plot.hpp"

using quickplot::CircularBuffer;
using quickplot::sync_right;

TEST(test_plot, sync_right_noop_if_synced)
{
  CircularBuffer b1(4);
  CircularBuffer b2(4);
  for (size_t i = 0; i < 4; i++) {
    auto pt = ImPlotPoint(i, i);
    b1.push_back(pt);
    b2.push_back(pt);
  }

  auto synced = sync_right(b1.begin(), b1.end(), b2.begin(), b2.end());
  ASSERT_EQ(synced.size(), 4ul);
  for (size_t i = 0; i < synced.size(); i++) {
    EXPECT_EQ(b2[i].y, synced[i]);
  }
}

TEST(test_plot, sync_right_fills_nans)
{
  CircularBuffer b1(4);
  CircularBuffer b2(3);
  for (size_t i = 0; i < 3; i++) {
    auto pt = ImPlotPoint(i, i);
    b1.push_back(pt);
    b2.push_back(pt);
  }
  b1.push_back(ImPlotPoint(3, 3));

  auto synced = sync_right(b1.begin(), b1.end(), b2.begin(), b2.end());
  ASSERT_EQ(synced.size(), 4ul);
  for (size_t i = 0; i < 3; i++)
  {
    EXPECT_EQ(synced[i], i);
  }
  EXPECT_TRUE(std::isnan(synced[3]));
}

TEST(test_plot, sync_right_delayed_start_nan_at_begin)
{
  CircularBuffer b1(4);
  CircularBuffer b2(3);
  b1.push_back(ImPlotPoint(0, 0));
  for (size_t i = 1; i < 4; i++) {
    auto pt = ImPlotPoint(i, i);
    b1.push_back(pt);
    b2.push_back(pt);
  }

  auto synced = sync_right(b1.begin(), b1.end(), b2.begin(), b2.end());
  ASSERT_EQ(synced.size(), 4ul);
  EXPECT_TRUE(std::isnan(synced[0]));
  for (size_t i = 1; i < 4; i++)
  {
    EXPECT_EQ(synced[i], b2[i - 1].y);
  }
}

TEST(test_plot, sync_right_empty_left_filled)
{
  CircularBuffer b1(4);
  CircularBuffer b2(0);
  for (size_t i = 0; i < 4; i++) {
    b1.push_back(ImPlotPoint(i, i));
  }
  auto synced = sync_right(b1.begin(), b1.end(), b2.begin(), b2.end());
  ASSERT_EQ(synced.size(), 4ul);
  for (size_t i = 0; i < 4; i++)
  {
    EXPECT_TRUE(std::isnan(synced[1]));
  }
}

TEST(test_plot, sync_right_dropped_entry_filled)
{
  CircularBuffer b1(4);
  CircularBuffer b2(4);
  for (auto i : {0, 1, 2, 3}) {
    b1.push_back(ImPlotPoint(i, i));
  }
  for (auto i : {0, 2, 3}) {
    b2.push_back(ImPlotPoint(i, i));
  }
  auto synced = sync_right(b1.begin(), b1.end(), b2.begin(), b2.end());
  ASSERT_EQ(synced.size(), 4ul);
  EXPECT_EQ(synced[0], 0.0);
  EXPECT_TRUE(std::isnan(synced[1]));
  EXPECT_EQ(synced[2], 2.0);
  EXPECT_EQ(synced[3], 3.0);
}
