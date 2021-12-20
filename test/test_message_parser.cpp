#include "quickplot/message_parser.hpp"
#include <gmock/gmock.h>
#include <memory>
#include <vector>
#include <string>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rosidl_typesupport_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>

using geometry_msgs::msg::TwistStamped;

TEST(test_parser, twist_stamped_parses_fields)
{
  auto introspection = std::make_shared<quickplot::MessageIntrospection>(
    "geometry_msgs/TwistStamped");
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

  auto stamp = deserializer.get_header_stamp(buffer.data()).value();
  EXPECT_EQ(stamp, rclcpp::Time(99, 99, RCL_ROS_TIME));

  // get initialized numeric
  auto linear_x_member = introspection->get_member({"twist", "linear", "x"});
  ASSERT_TRUE(linear_x_member.has_value());
  auto linear_x = quickplot::get_numeric(buffer.data(), linear_x_member.value());
  EXPECT_FLOAT_EQ(linear_x, 1.0);

  // get default-initialized numeric
  auto linear_y_member = introspection->get_member({"twist", "linear", "y"});
  ASSERT_TRUE(linear_y_member.has_value());
  auto linear_y = quickplot::get_numeric(buffer.data(), linear_y_member.value());
  EXPECT_FLOAT_EQ(linear_y, 0.0);
}
