#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <cmath>
#include <string>
#include <vector>
#include <memory>
#include "quickplot/introspection.hpp"
#include "quickplot/message_parser.hpp"

namespace quickplot
{

class IntrospectionMessageDeserializer
{
private:
  std::shared_ptr<MessageIntrospection> introspection_;
  std::shared_ptr<rcpputils::SharedLibrary> type_support_library_;
  const rosidl_message_type_support_t * type_support_handle_;
  std::unique_ptr<rclcpp::SerializationBase> deserializer_;
  std::optional<size_t> header_offset_;

public:
  explicit IntrospectionMessageDeserializer(std::shared_ptr<MessageIntrospection> introspection);

  // disable copy and move
  IntrospectionMessageDeserializer & operator=(IntrospectionMessageDeserializer && other) = delete;

  const char* message_type() const;

  std::vector<uint8_t> init_buffer() const;

  void fini_buffer(std::vector<uint8_t> & buffer) const;

  void deserialize(const rclcpp::SerializedMessage &, void * message) const;

  std::optional<rclcpp::Time> get_header_stamp(void * message) const;
};

} // namespace quickplot
