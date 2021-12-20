#include <memory>
#include <string>
#include <vector>
#include <deque>
#include <exception>
#include <std_msgs/msg/header.hpp>
#include <rosidl_typesupport_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include "quickplot/introspection.hpp"
#include "quickplot/message_parser.hpp"

namespace quickplot
{

IntrospectionMessageDeserializer::IntrospectionMessageDeserializer(
  std::shared_ptr<MessageIntrospection> introspection)
: introspection_(introspection)
{
  type_support_library_ = rclcpp::get_typesupport_library(
    introspection_->message_type(),
    rosidl_typesupport_cpp::typesupport_identifier);
  type_support_handle_ = rclcpp::get_typesupport_handle(
    introspection_->message_type(),
    rosidl_typesupport_cpp::typesupport_identifier,
    *type_support_library_);

  deserializer_ = std::make_unique<rclcpp::SerializationBase>(type_support_handle_);
  header_offset_ = introspection_->get_header_offset();
}

const char * IntrospectionMessageDeserializer::message_type() const
{
  return introspection_->message_type();
}

std::vector<uint8_t> IntrospectionMessageDeserializer::init_buffer() const
{
  std::vector<uint8_t> buffer;
  auto members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
    introspection_->get_typesupport_handle()->data);
  buffer.resize(members->size_of_);
  members->init_function(
    buffer.data(),
    rosidl_runtime_cpp::MessageInitialization::ALL);
  return buffer;
}

void IntrospectionMessageDeserializer::fini_buffer(std::vector<uint8_t> & buffer) const
{
  auto members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
    introspection_->get_typesupport_handle()->data);
  members->fini_function(buffer.data());
}

void IntrospectionMessageDeserializer::deserialize(
  const rclcpp::SerializedMessage & serialized_message,
  void * message) const
{
  deserializer_->deserialize_message(&serialized_message, message);
}

std::optional<rclcpp::Time> IntrospectionMessageDeserializer::get_header_stamp(void * message) const
{
  if (!header_offset_.has_value()) {
    return {};
  }
  auto bytes = static_cast<uint8_t *>(message);
  auto header =
    static_cast<std_msgs::msg::Header *>(static_cast<void *>(bytes + header_offset_.value()));
  return header->stamp;
}

} // namespace quickplot
