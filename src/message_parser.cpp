#include <memory>
#include <string>
#include <vector>
#include <std_msgs/msg/header.hpp>
#include <rosidl_typesupport_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include "quickplot/message_parser.hpp"

namespace quickplot
{

double cast_numeric(void * message, MemberInfo info)
{
  auto n = static_cast<void *>(static_cast<uint8_t *>(message) + info.offset);
  switch (info.type_id) {
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

std::optional<size_t> find_header_offset(
  const rosidl_message_type_support_t * introspection_support)
{
  auto members =
    static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(introspection_support
    ->data);
  for (size_t i = 0; i < members->member_count_; i++) {
    // search only on first level of the member tree
    const auto & member = members->members_[i];
    if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
      auto sub_members =
        static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(member.
        members_->data);
      if (strcmp("header", member.name_) == 0) {
        if (strcmp(sub_members->message_name_, "Header") == 0 && strcmp(
            sub_members->message_namespace_, "std_msgs::msg") == 0)
        {
          return member.offset_;
        }
      }
    }
  }
  return {};
}

bool is_numeric(uint8_t type_id)
{
  switch (type_id) {
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT:
      return true;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE:
      return true;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
      return true;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
      return true;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
      return true;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
      return true;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
      return true;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
      return true;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
      return true;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
      return true;
    default:
      return false;
  }
}

std::optional<MemberInfo> _find_member(
  const rosidl_message_type_support_t * introspection_support,
  size_t depth,
  std::vector<std::string>::const_iterator target_path_it,
  std::vector<std::string>::const_iterator target_path_end,
  int64_t parent_offset)
{
  assert(target_path_it != target_path_end);
  auto members =
    static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(introspection_support
    ->data);

  for (size_t i = 0; i < members->member_count_; i++) {
    const auto & member = members->members_[i];
    if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
      if (target_path_end == (target_path_it + 1)) {
        // if target path is a single element, the member must be on the current level of the tree
        // thus, we can early-out when we see a nested message
        continue;
      }
      if (target_path_it->compare(member.name_) != 0) {
        continue;
      }
      auto info = _find_member(
        member.members_, depth + 1, ++target_path_it, target_path_end,
        parent_offset + member.offset_);
      --target_path_it;
      if (info.has_value()) {
        return info;
      }
    } else if (target_path_end == (target_path_it + 1)) {
      // match target_path with leaf members only if the remaining path is a single member name
      if (target_path_it->compare(member.name_) != 0) {
        continue;
      }
      if (!is_numeric(member.type_id_)) {
        throw std::invalid_argument("Requested message member is not numeric");
      }
      return MemberInfo {
        .offset = (parent_offset + member.offset_),
        .type_id = member.type_id_
      };
    }
  }
  return {};
}

IntrospectionMessageDeserializer::IntrospectionMessageDeserializer(std::string topic_type)
: topic_type_(topic_type)
{
  type_support_library_ = rclcpp::get_typesupport_library(
    topic_type,
    rosidl_typesupport_cpp::typesupport_identifier);
  type_support_handle_ = rclcpp::get_typesupport_handle(
    topic_type,
    rosidl_typesupport_cpp::typesupport_identifier,
    *type_support_library_);

  introspection_support_library_ = rclcpp::get_typesupport_library(
    topic_type,
    rosidl_typesupport_introspection_cpp::typesupport_identifier);
  introspection_support_handle_ = rclcpp::get_typesupport_handle(
    topic_type,
    rosidl_typesupport_introspection_cpp::typesupport_identifier,
    *introspection_support_library_);
  deserializer_ = std::make_unique<rclcpp::SerializationBase>(type_support_handle_);
  header_offset_ = find_header_offset(introspection_support_handle_);
}

std::string IntrospectionMessageDeserializer::topic_type() const
{
  return topic_type_;
}

std::vector<uint8_t> IntrospectionMessageDeserializer::init_buffer() const
{
  std::vector<uint8_t> buffer;
  auto members =
    static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
    introspection_support_handle_
    ->data);
  buffer.resize(members->size_of_);
  members->init_function(
    buffer.data(),
    rosidl_runtime_cpp::MessageInitialization::ALL);
  return buffer;
}

void IntrospectionMessageDeserializer::fini_buffer(std::vector<uint8_t> & buffer) const
{
  auto members =
    static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
    introspection_support_handle_
    ->data);
  members->fini_function(buffer.data());
}

void IntrospectionMessageDeserializer::deserialize(
  const rclcpp::SerializedMessage & message,
  void * buffer) const
{
  deserializer_->deserialize_message(&message, buffer);
}


rclcpp::Time IntrospectionMessageDeserializer::get_header_stamp(void * buffer) const
{
  assert(header_offset_.has_value());
  auto bytes = static_cast<uint8_t *>(buffer);
  auto header =
    static_cast<std_msgs::msg::Header *>(static_cast<void *>(bytes + header_offset_.value()));
  return header->stamp;
}

std::optional<MemberInfo> IntrospectionMessageDeserializer::find_member(
  std::vector<std::string> member_path) const
{
  if (member_path.empty()) {
    std::invalid_argument("member_path required");
  }
  auto initial_members =
    static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
    introspection_support_handle_->data);

  if (initial_members->member_count_ <= 0) {
    return {};
  }
  return _find_member(
    introspection_support_handle_, 0, member_path.cbegin(), member_path.cend(), 0);
}

} // namespace quickplot
