#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <functional>
#include <cmath>
#include <string>
#include <vector>
#include <deque>
#include <std_msgs/msg/header.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

namespace quickplot
{

double cast_numeric_to_double(uint8_t type_id, void * n)
{
  switch (type_id) {
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT:
      return *static_cast<float *>(n);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE:
      return *static_cast<double *>(n);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
      return *static_cast<int64_t *>(n);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
      return *static_cast<int32_t *>(n);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
      return *static_cast<int16_t *>(n);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
      return *static_cast<int8_t *>(n);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
      return *static_cast<uint64_t *>(n);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
      return *static_cast<uint32_t *>(n);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
      return *static_cast<uint16_t *>(n);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
      return *static_cast<uint8_t *>(n);
    default:
      throw std::invalid_argument("Unknown member type_id");
  }
}

size_t offset_of_type_id(uint8_t type_id)
{

  switch (type_id) {
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT:
      return sizeof(float);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE:
      return sizeof(double);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
      return sizeof(int64_t);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
      return sizeof(int32_t);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
      return sizeof(int16_t);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
      return sizeof(int8_t);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
      return sizeof(uint64_t);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
      return sizeof(uint32_t);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
      return sizeof(uint16_t);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
      return sizeof(uint8_t);
    default:
      throw std::invalid_argument("Unknown member type_id");
  }
}

struct MessageFieldData
{
  std::vector<std::string> member_path;
  double value;
};

struct MessageDataBuffer
{
  rclcpp::Time stamp;
  std::vector<MessageFieldData> data;
};

struct StackItem
{
  const rosidl_typesupport_introspection_cpp::MessageMembers * members;
  size_t member_index;
  uint8_t * message_start;

  const char * current_name() const
  {
    return members->members_[member_index].name_;
  }

  const rosidl_typesupport_introspection_cpp::MessageMember * current_member() const
  {
    return &members->members_[member_index];
  }
};

void _parse_generic_message(
  std::deque<std::string> member_path,
  const rosidl_message_type_support_t * introspection_support,
  uint8_t * message_part,
  MessageDataBuffer & output_buffer)
{
  auto members =
    static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(introspection_support
    ->data);

  for (size_t i = 0; i < members->member_count_; i++) {
    const auto & member = members->members_[i];
    if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
      auto sub_members =
        static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(member.
        members_->data);
      // special handling for header, to read time stamp
      bool handled = false;
      if (member_path.size() == 0) {
        if (strcmp("header", member.name_) == 0) {
          if (strcmp(sub_members->message_name_, "Header") == 0 && strcmp(
              sub_members->message_namespace_, "std_msgs::msg") == 0)
          {
            auto header = static_cast<std_msgs::msg::Header *>(static_cast<void *>(message_part));
            output_buffer.stamp = header->stamp;
            handled = true;
          }
        }
      }
      if (!handled) {
        member_path.push_back(member.name_);
        _parse_generic_message(
          member_path, member.members_, message_part + member.offset_,
          output_buffer);
        member_path.pop_back();
      }
    } else {
      // attempt to find a requested field that matches in full member path
      for (auto & field: output_buffer.data) {
        if (field.member_path.size() != member_path.size() + 1) {
          continue;
        }
        if (field.member_path.back().compare(member.name_) != 0) {
          continue;
        }
        bool matches = true;
        for (size_t i = 0; i < field.member_path.size() - 1; i++) {
          if (field.member_path[i].compare(member_path[i]) != 0) {
            matches = false;
            break;
          }
        }
        if (!matches) {
          continue;
        }
        field.value = cast_numeric_to_double(member.type_id_, message_part + member.offset_);
        break;
      }
    }
  }
}

void parse_generic_message(
  const rosidl_message_type_support_t * type_support,
  const rosidl_message_type_support_t * introspection_support,
  const rclcpp::SerializedMessage & serialized_message,
  void * buffer_message,
  MessageDataBuffer & output_buffer)
{
  auto initial_members =
    static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
    introspection_support
    ->data);

  if (initial_members->member_count_ <= 0) {
    return;
  }

  for (const auto & field: output_buffer.data) {
    if (field.member_path.empty()) {
      std::invalid_argument("Member path for every field must be set");
    }
  }

  rclcpp::SerializationBase deserializer(type_support);
  deserializer.deserialize_message(&serialized_message, buffer_message);

  std::deque<std::string> member_path;
  _parse_generic_message(
    member_path,
    introspection_support, static_cast<uint8_t *>(buffer_message),
    output_buffer);
}

} // namespace quickplot
