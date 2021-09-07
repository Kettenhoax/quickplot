#include "quickplot/message_parser.hpp"
#include <gmock/gmock.h>
#include <memory>
#include <vector>
#include <algorithm>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rosidl_typesupport_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>

using geometry_msgs::msg::TwistStamped;
using nav_msgs::msg::Odometry;

TEST(test_parser, twist_get_member_infos_lists_numeric_members)
{
  auto topic_type = "geometry_msgs/Twist";
  auto introspection = std::make_shared<quickplot::MessageIntrospection>(topic_type);
  auto c_members = introspection->members();
  std::vector<quickplot::MessageMember> members(c_members.begin(), c_members.end());

  EXPECT_EQ(members.size(), 6lu);
  EXPECT_EQ(members[0].info.type_id, rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT64);
  ASSERT_THAT(members[0].path, ::testing::ElementsAre("linear", "x"));
  EXPECT_EQ(members[1].info.type_id, rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT64);
  ASSERT_THAT(members[1].path, ::testing::ElementsAre("linear", "y"));
  EXPECT_EQ(members[2].info.type_id, rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT64);
  ASSERT_THAT(members[2].path, ::testing::ElementsAre("linear", "z"));

  EXPECT_EQ(members[3].info.type_id, rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT64);
  ASSERT_THAT(members[3].path, ::testing::ElementsAre("angular", "x"));
  EXPECT_EQ(members[4].info.type_id, rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT64);
  ASSERT_THAT(members[4].path, ::testing::ElementsAre("angular", "y"));
  EXPECT_EQ(members[5].info.type_id, rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT64);
  ASSERT_THAT(members[5].path, ::testing::ElementsAre("angular", "z"));
}

TEST(test_parser, twist_stamped_get_member_infos_lists_single_stamp)
{
  auto topic_type = "geometry_msgs/TwistStamped";
  auto introspection = std::make_shared<quickplot::MessageIntrospection>(topic_type);
  auto c_members = introspection->members();
  std::vector<quickplot::MessageMember> members(c_members.begin(), c_members.end());

  EXPECT_EQ(members.size(), 7lu);
  EXPECT_EQ(members[0].info.type_id, rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE);
  ASSERT_THAT(members[0].path, ::testing::ElementsAre("header", "stamp"));
}

TEST(test_parser, parse_twist_stamped_header_and_field)
{
  auto topic_type = "geometry_msgs/TwistStamped";
  auto introspection = std::make_shared<quickplot::MessageIntrospection>(topic_type);
  quickplot::IntrospectionMessageDeserializer deserializer(introspection);

  TwistStamped msg;
  msg.header.stamp = rclcpp::Time(99, 99, RCL_ROS_TIME);
  msg.twist.linear.x = 1.0;

  rclcpp::Serialization<TwistStamped> serializer;
  rclcpp::SerializedMessage serialized_msg;
  serializer.serialize_message(static_cast<void *>(&msg), &serialized_msg);

  auto buffer = deserializer.init_buffer();
  deserializer.deserialize(serialized_msg, buffer.data());
  deserializer.fini_buffer(buffer);

  auto member_info = introspection->get_member_info({"twist", "linear", "x"});
  EXPECT_TRUE(member_info.has_value());
  auto stamp = deserializer.get_header_stamp(buffer.data()).value();
  EXPECT_EQ(stamp, rclcpp::Time(99, 99, RCL_ROS_TIME));
  auto twist_linear_x_value = deserializer.get_numeric(buffer.data(), member_info.value());
  EXPECT_EQ(twist_linear_x_value, 1.0);
}

TEST(test_parser, parse_odometry_header_and_field)
{
  // test is different from Twist, since Odometry contains an array
  auto topic_type = "nav_msgs/Odometry";
  auto introspection = std::make_shared<quickplot::MessageIntrospection>(topic_type);
  quickplot::IntrospectionMessageDeserializer deserializer(introspection);

  Odometry msg;
  msg.header.stamp = rclcpp::Time(10, 1e9 - 1, RCL_ROS_TIME);
  msg.pose.pose.position.y = 1.0;
  msg.pose.pose.orientation.w = 2.0;

  rclcpp::Serialization<Odometry> serializer;
  rclcpp::SerializedMessage serialized_msg;
  serializer.serialize_message(static_cast<void *>(&msg), &serialized_msg);

  auto buffer = deserializer.init_buffer();
  deserializer.deserialize(serialized_msg, buffer.data());
  deserializer.fini_buffer(buffer);

  auto position_y_member = introspection->get_member_info({"pose", "pose", "position", "y"});
  EXPECT_TRUE(position_y_member.has_value());

  auto orientation_w_member = introspection->get_member_info({"pose", "pose", "orientation", "w"});
  EXPECT_TRUE(orientation_w_member.has_value());

  auto stamp = deserializer.get_header_stamp(buffer.data()).value();
  EXPECT_EQ(stamp, rclcpp::Time(10, 1e9 - 1, RCL_ROS_TIME));

  auto position_y_value = deserializer.get_numeric(buffer.data(), position_y_member.value());
  EXPECT_EQ(position_y_value, 1.0);

  auto orientation_w_value = deserializer.get_numeric(buffer.data(), orientation_w_member.value());
  EXPECT_EQ(orientation_w_value, 2.0);
}
