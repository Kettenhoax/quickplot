#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <cmath>
#include <string>
#include <vector>
#include <memory>

namespace quickplot
{

struct MemberInfo {
  // -1 if not found, offset into message otherwise
  int64_t offset;
  uint8_t type_id;
};

double cast_numeric(void * message, MemberInfo);

class IntrospectionMessageDeserializer {
private:
  std::string topic_type_;
  std::shared_ptr<rcpputils::SharedLibrary> type_support_library_;
  std::shared_ptr<rcpputils::SharedLibrary> introspection_support_library_;
  const rosidl_message_type_support_t * type_support_handle_;
  const rosidl_message_type_support_t * introspection_support_handle_;
  std::unique_ptr<rclcpp::SerializationBase> deserializer_;
  std::optional<size_t> header_offset_;
public:
  explicit IntrospectionMessageDeserializer(std::string topic_type);

  // disable copy and move
  IntrospectionMessageDeserializer & operator=(IntrospectionMessageDeserializer && other) = delete;
  std::string topic_type() const;

  std::vector<uint8_t> init_buffer() const;
  void fini_buffer(std::vector<uint8_t> & buffer) const;
  void deserialize(const rclcpp::SerializedMessage& message, void * buffer) const;

  rclcpp::Time get_header_stamp(void* buffer) const;

  std::optional<MemberInfo> find_member(std::vector<std::string> member_path) const;
};

} // namespace quickplot
