#include "quickplot/message_parser.hpp"
#include <gmock/gmock.h>
#include <memory>
#include <vector>
#include <string>
#include <utility>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <rosidl_typesupport_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>

using MB = quickplot::MemberSequencePathItemDescriptor;
static MB mb(std::string member_name)
{
  return MB {
    member_name,
    std::nullopt
  };
}
static MB mbi(std::string member_name, size_t i)
{
  return MB {
    member_name,
    i
  };
}

TEST(test_message_parser, twist_stamped_parses_fields)
{
  using geometry_msgs::msg::TwistStamped;
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

  auto stamp = deserializer.get_header_stamp(buffer.data()).value();
  EXPECT_EQ(stamp, rclcpp::Time(99, 99, RCL_ROS_TIME));

  // get initialized numeric
  auto linear_x_member = introspection->get_member_sequence_path(
    {mb("twist"), mb("linear"), mb("x")});
  ASSERT_TRUE(linear_x_member.has_value());
  auto linear_x = quickplot::get_numeric(buffer.data(), linear_x_member.value());
  EXPECT_FLOAT_EQ(linear_x, 1.0);

  // get default-initialized numeric
  auto linear_y_member = introspection->get_member_sequence_path(
    {mb("twist"), mb("linear"), mb("y")});
  ASSERT_TRUE(linear_y_member.has_value());
  auto linear_y = quickplot::get_numeric(buffer.data(), linear_y_member.value());
  EXPECT_FLOAT_EQ(linear_y, 0.0);

  deserializer.fini_buffer(buffer);
}

TEST(test_message_parser, twist_applies_sqrt_operator)
{
  using geometry_msgs::msg::Twist;
  auto introspection = std::make_shared<quickplot::MessageIntrospection>("geometry_msgs/Twist");
  quickplot::IntrospectionMessageDeserializer deserializer(introspection);

  Twist msg;
  msg.linear.x = 4.0;
  rclcpp::Serialization<Twist> serializer;
  rclcpp::SerializedMessage serialized_msg;
  serializer.serialize_message(static_cast<void *>(&msg), &serialized_msg);

  auto buffer = deserializer.init_buffer();
  deserializer.deserialize(serialized_msg, buffer.data());

  // get initialized numeric
  auto linear_x_member = introspection->get_member_sequence_path({mb("linear"), mb("x")});
  ASSERT_TRUE(linear_x_member.has_value());
  auto linear_x = quickplot::get_numeric(
    buffer.data(),
    linear_x_member.value(), quickplot::DataSourceOperator::Sqrt);
  EXPECT_FLOAT_EQ(linear_x, 2.0);

  deserializer.fini_buffer(buffer);
}

TEST(test_message_parser, detection_parses_nested_array)
{
  using vision_msgs::msg::Detection3DArray;
  auto introspection = std::make_shared<quickplot::MessageIntrospection>(
    "vision_msgs/Detection3DArray");
  quickplot::IntrospectionMessageDeserializer deserializer(introspection);

  Detection3DArray msg;
  msg.detections.resize(2);
  msg.detections[0].results.resize(1);
  msg.detections[0].results[0].hypothesis.score = 2.0;
  msg.detections[0].results[0].pose.covariance[35] = 3.0;
  msg.detections[1].bbox.center.position.x = 1.0;

  rclcpp::Serialization<Detection3DArray> serializer;
  rclcpp::SerializedMessage serialized_msg;
  serializer.serialize_message(static_cast<void *>(&msg), &serialized_msg);

  auto buffer = deserializer.init_buffer();
  deserializer.deserialize(serialized_msg, buffer.data());

  // singly nested, field leaf
  auto x_member_opt =
    introspection->get_member_sequence_path(
    {mbi("detections", 1), mb("bbox"), mb("center"), mb("position"), mb("x")});
  ASSERT_TRUE(x_member_opt.has_value());
  auto box_x = quickplot::get_numeric(buffer.data(), x_member_opt.value());
  EXPECT_FLOAT_EQ(box_x, 1.0);

  // doubly nested, field leaf
  auto score_member_opt =
    introspection->get_member_sequence_path(
    {mbi("detections", 0), mbi("results", 0), mb("hypothesis"), mb("score")});
  ASSERT_TRUE(score_member_opt.has_value());
  auto score = quickplot::get_numeric(buffer.data(), score_member_opt.value());
  EXPECT_FLOAT_EQ(score, 2.0);

  // triple nested, array leaf
  auto cov_member_opt = introspection->get_member_sequence_path(
    {mbi("detections", 0), mbi("results", 0), mb("pose"), mbi("covariance", 35)});
  ASSERT_TRUE(cov_member_opt.has_value());
  auto cov = quickplot::get_numeric(buffer.data(), cov_member_opt.value());
  EXPECT_FLOAT_EQ(cov, 3.0);

  deserializer.fini_buffer(buffer);
}
