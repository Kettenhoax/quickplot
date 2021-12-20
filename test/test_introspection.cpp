#include "quickplot/introspection.hpp"
#include <gmock/gmock.h>
#include <memory>
#include <vector>
#include <string>
#include <rosidl_typesupport_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>

using ::testing::StrEq;

void check_path(const quickplot::MemberPath& path, std::initializer_list<std::string> names)
{
  auto path_it = path.begin();
  for(const auto& name: names) {
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
  auto introspection = std::make_shared<quickplot::MessageIntrospection>("geometry_msgs/TwistStamped");
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

TEST(test_introspection, fmt_member_formats_nested)
{
  auto introspection = std::make_shared<quickplot::MessageIntrospection>("geometry_msgs/TwistStamped");
  auto member_path = introspection->get_member({"twist", "linear", "z"});
  ASSERT_TRUE(member_path.has_value());
  EXPECT_THAT(quickplot::fmt_member_path(member_path.value()), StrEq("twist.linear.z"));
}