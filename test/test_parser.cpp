#include "quickplot/parser.hpp"
#include "quickplot/typesupport_helpers.hpp"
#include <gtest/gtest.h>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rosidl_typesupport_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>

using geometry_msgs::msg::TwistStamped;

TEST(test_parser, parse_twist_stamped_header_and_field)
{
  auto topic_type = "geometry_msgs/TwistStamped";

  auto type_support_library = quickplot::get_typesupport_library(
    topic_type,
    rosidl_typesupport_cpp::typesupport_identifier);
  auto type_support = quickplot::get_typesupport_handle(
    topic_type,
    rosidl_typesupport_cpp::typesupport_identifier,
    type_support_library);

  auto introspection_library = quickplot::get_typesupport_library(
    topic_type,
    rosidl_typesupport_introspection_cpp::typesupport_identifier);
  auto introspection_support = quickplot::get_typesupport_handle(
    topic_type,
    rosidl_typesupport_introspection_cpp::typesupport_identifier,
    introspection_library);

  TwistStamped msg;
  msg.header.stamp = rclcpp::Time(99, 99, RCL_ROS_TIME);
  msg.twist.linear.x = 1.0;

  rclcpp::Serialization<TwistStamped> serializer;
  rclcpp::SerializedMessage serialized_msg;
  serializer.serialize_message(static_cast<void*>(&msg), &serialized_msg);

  TwistStamped buffer_msg;

  quickplot::MessageDataBuffer buffer;
  auto & linear_x_data = buffer.data.emplace_back();
  linear_x_data.member_path = {"twist", "linear", "x"};

  quickplot::parse_generic_message(
    type_support, introspection_support,
    serialized_msg, static_cast<void*>(&buffer_msg), buffer);

  EXPECT_EQ(buffer.stamp, rclcpp::Time(99, 99, RCL_ROS_TIME));
  EXPECT_EQ(buffer.data.size(), 1ul);
  EXPECT_EQ(buffer.data[0].value, 1.0);
}
