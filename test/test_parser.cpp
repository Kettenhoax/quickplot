#include "quickplot/message_parser.hpp"
#include <gtest/gtest.h>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rosidl_typesupport_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>

using geometry_msgs::msg::TwistStamped;
using nav_msgs::msg::Odometry;

TEST(test_parser, parse_twist_stamped_header_and_field)
{
  auto topic_type = "geometry_msgs/TwistStamped";
  quickplot::IntrospectionMessageDeserializer deserializer(topic_type);

  TwistStamped msg;
  msg.header.stamp = rclcpp::Time(99, 99, RCL_ROS_TIME);
  msg.twist.linear.x = 1.0;

  rclcpp::Serialization<TwistStamped> serializer;
  rclcpp::SerializedMessage serialized_msg;
  serializer.serialize_message(static_cast<void*>(&msg), &serialized_msg);

  auto buffer = deserializer.init_buffer();
  deserializer.deserialize(serialized_msg, buffer.data());
  deserializer.fini_buffer(buffer);

  auto member_info = deserializer.find_member({"twist", "linear", "x"});
  EXPECT_TRUE(member_info.has_value());
  auto stamp = deserializer.get_header_stamp(buffer.data());
  EXPECT_EQ(stamp, rclcpp::Time(99, 99, RCL_ROS_TIME));
  auto twist_linear_x_value = quickplot::cast_numeric(buffer.data(), member_info.value());
  EXPECT_EQ(twist_linear_x_value, 1.0);
}

TEST(test_parser, parse_odometry_header_and_field)
{
  // test is different from Twist, since Odometry contains an array
  auto topic_type = "nav_msgs/Odometry";
  quickplot::IntrospectionMessageDeserializer deserializer(topic_type);

  Odometry msg;
  msg.header.stamp = rclcpp::Time(10, 1e9 - 1, RCL_ROS_TIME);
  msg.pose.pose.position.y = 1.0;
  msg.pose.pose.orientation.w = 2.0;

  rclcpp::Serialization<Odometry> serializer;
  rclcpp::SerializedMessage serialized_msg;
  serializer.serialize_message(static_cast<void*>(&msg), &serialized_msg);

  auto buffer = deserializer.init_buffer();
  deserializer.deserialize(serialized_msg, buffer.data());
  deserializer.fini_buffer(buffer);

  auto position_y_member = deserializer.find_member({"pose", "pose", "position", "y"});
  EXPECT_TRUE(position_y_member.has_value());

  auto orientation_w_member = deserializer.find_member({"pose", "pose", "orientation", "w"});
  EXPECT_TRUE(orientation_w_member.has_value());

  auto stamp = deserializer.get_header_stamp(buffer.data());
  EXPECT_EQ(stamp, rclcpp::Time(10, 1e9 - 1, RCL_ROS_TIME));
  
  auto position_y_value = quickplot::cast_numeric(buffer.data(), position_y_member.value());
  EXPECT_EQ(position_y_value, 1.0);

  auto orientation_w_value = quickplot::cast_numeric(buffer.data(), orientation_w_member.value());
  EXPECT_EQ(orientation_w_value, 2.0);
}
