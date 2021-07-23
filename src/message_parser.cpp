#include <memory>
#include <string>
#include <vector>
#include <deque>
#include <std_msgs/msg/header.hpp>
#include <rosidl_typesupport_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include "quickplot/message_parser.hpp"

namespace quickplot
{

double cast_numeric(void * n, uint8_t type_id)
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

std::optional<MessageMemberInfo> _find_member(
  const rosidl_message_type_support_t * introspection_support,
  size_t depth,
  std::vector<std::string>::const_iterator target_path_it,
  std::vector<std::string>::const_iterator target_path_end,
  uint32_t parent_offset)
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
      return MessageMemberInfo {
        .offset = (parent_offset + member.offset_),
        .type_id = member.type_id_
      };
    }
  }
  return {};
}

MessageIntrospection::MessageIntrospection(std::string message_type) : message_type_(message_type)
{
  introspection_support_library_ = rclcpp::get_typesupport_library(
    message_type,
    rosidl_typesupport_introspection_cpp::typesupport_identifier);
  introspection_support_handle_ = rclcpp::get_typesupport_handle(
    message_type,
    rosidl_typesupport_introspection_cpp::typesupport_identifier,
    *introspection_support_library_);
}

std::string MessageIntrospection::message_type() const
{
  return message_type_;
}

const rosidl_typesupport_introspection_cpp::MessageMembers * MessageIntrospection::members() const
{
  return static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
    introspection_support_handle_->data);
}

std::optional<size_t> MessageIntrospection::get_header_offset() const
{
  auto m = members();
  for (size_t i = 0; i < m->member_count_; i++) {
    // search only on first level of the member tree
    const auto & member = m->members_[i];
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

MemberIterator MessageIntrospection::begin_member_infos() const
{
  return MemberIterator(introspection_support_handle_);
}

MemberIterator MessageIntrospection::end_member_infos() const
{
  return MemberIterator(nullptr);
}

std::optional<MessageMemberInfo> MessageIntrospection::get_member_info(
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

std::string IntrospectionMessageDeserializer::message_type() const
{
  return introspection_->message_type();
}

std::vector<uint8_t> IntrospectionMessageDeserializer::init_buffer() const
{
  std::vector<uint8_t> buffer;
  auto members = introspection_->members();
  buffer.resize(members->size_of_);
  members->init_function(
    buffer.data(),
    rosidl_runtime_cpp::MessageInitialization::ALL);
  return buffer;
}

void IntrospectionMessageDeserializer::fini_buffer(std::vector<uint8_t> & buffer) const
{
  introspection_->members()->fini_function(buffer.data());
}

void IntrospectionMessageDeserializer::deserialize(
  const rclcpp::SerializedMessage & serialized_message,
  void * message) const
{
  deserializer_->deserialize_message(&serialized_message, message);
}

rclcpp::Time IntrospectionMessageDeserializer::get_header_stamp(void * message) const
{
  assert(header_offset_.has_value());
  auto bytes = static_cast<uint8_t *>(message);
  auto header =
    static_cast<std_msgs::msg::Header *>(static_cast<void *>(bytes + header_offset_.value()));
  return header->stamp;
}

double IntrospectionMessageDeserializer::get_numeric(void * message, MessageMemberInfo info) const
{
  auto bytes = reinterpret_cast<uint8_t *>(message) + info.offset;
  return cast_numeric(bytes, info.type_id);
}

} // namespace quickplot
