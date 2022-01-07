#include "quickplot/introspection.hpp"
#include <gmock/gmock.h>
#include <memory>
#include <vector>
#include <string>
#include <rosidl_typesupport_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>

using ::testing::StrEq;

void check_path(const quickplot::MemberPath & path, std::initializer_list<std::string> names)
{
  auto path_it = path.begin();
  for (const auto & name: names) {
    ASSERT_FALSE(path_it == path.end());
    EXPECT_THAT((*path_it)->name_, StrEq(name));
    ++path_it;
  }
}

TEST(test_introspection, twist_member_iterator_lists_members)
{
  auto introspection = std::make_shared<quickplot::MessageIntrospection>("geometry_msgs/Twist");
  auto members = introspection->members();
  auto it = members.begin();
  check_path(*it, {"linear"});
  ++it;
  check_path(*it, {"linear", "x"});
  ++it;
  check_path(*it, {"linear", "y"});
  ++it;
  check_path(*it, {"linear", "z"});
  ++it;
  check_path(*it, {"angular"});
  ++it;
  check_path(*it, {"angular", "x"});
  ++it;
  check_path(*it, {"angular", "y"});
  ++it;
  check_path(*it, {"angular", "z"});
  ++it;
  EXPECT_FALSE(it != members.end());
}

TEST(test_introspection, twist_stamped_member_iterator_lists_doubly_nested_members)
{
  auto introspection = std::make_shared<quickplot::MessageIntrospection>(
    "geometry_msgs/TwistStamped");
  auto members = introspection->members();
  auto it = members.begin();
  EXPECT_FALSE(it->empty());
  check_path(*it, {"header"});
  ++it;
  check_path(*it, {"header", "stamp"});
  ++it;
  check_path(*it, {"header", "stamp", "sec"});
  ++it;
  check_path(*it, {"header", "stamp", "nanosec"});
  ++it;
  check_path(*it, {"header", "frame_id"});
  ++it;
  check_path(*it, {"twist"});
  ++it;
  check_path(*it, {"twist", "linear"});
  ++it;
  check_path(*it, {"twist", "linear", "x"});
  ++it;
  check_path(*it, {"twist", "linear", "y"});
  ++it;
  check_path(*it, {"twist", "linear", "z"});
  ++it;
  check_path(*it, {"twist", "angular"});
  ++it;
  check_path(*it, {"twist", "angular", "x"});
  ++it;
  check_path(*it, {"twist", "angular", "y"});
  ++it;
  check_path(*it, {"twist", "angular", "z"});
  ++it;
  EXPECT_FALSE(it != members.end());
}

TEST(test_introspection, pose_with_covariance_stamped_array_access)
{
  auto introspection = std::make_shared<quickplot::MessageIntrospection>(
    "geometry_msgs/PoseWithCovarianceStamped");
  auto member_path_opt = introspection->get_member_sequence_path(
    {{"pose", std::nullopt}, {"covariance", 1}});
  ASSERT_TRUE(member_path_opt.has_value());
  auto member_path = member_path_opt.value();
  EXPECT_EQ(member_path.size(), 2ul);
  EXPECT_THAT(member_path[0].first->name_, StrEq("pose"));
  EXPECT_EQ(member_path[0].second, 0ul);
  EXPECT_THAT(member_path[1].first->name_, StrEq("covariance"));
  EXPECT_EQ(member_path[1].second, 1ul);
}

TEST(test_introspection, index_in_descriptor_for_non_sequence_member_fails)
{
  auto introspection = std::make_shared<quickplot::MessageIntrospection>(
    "geometry_msgs/PoseWithCovarianceStamped");
  EXPECT_THROW(
  {
    auto member_path_opt = introspection->get_member_sequence_path({{"pose", 0}});
  }, quickplot::introspection_error);
}

TEST(test_introspection, stream_insert_members)
{
  auto introspection = std::make_shared<quickplot::MessageIntrospection>(
    "geometry_msgs/TwistStamped");
  auto member_path = introspection->get_member_sequence_path({{"twist", std::nullopt}, {"linear",
          std::nullopt}, {"z", std::nullopt}});
  ASSERT_TRUE(member_path.has_value());
  std::stringstream ss;
  ss << member_path.value();
  EXPECT_THAT(ss.str(), StrEq("twist.linear.z"));

  ss = std::stringstream();
  ss << quickplot::to_descriptor(member_path.value());
  EXPECT_THAT(ss.str(), StrEq("twist.linear.z"));
}
