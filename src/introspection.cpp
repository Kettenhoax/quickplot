#include <vector>
#include <stdexcept>
#include <string>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <boost/utility.hpp> // for next(it)
#include <geometry_msgs/msg/vector3.hpp>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>
#include "quickplot/introspection.hpp"

namespace quickplot
{

using geometry_msgs::msg::Vector3;

double cast_numeric(const void * n, uint8_t type_id)
{
  switch (type_id) {
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT:
      return *static_cast<const float *>(n);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE:
      return *static_cast<const double *>(n);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
      return *static_cast<const int64_t *>(n);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
      return *static_cast<const int32_t *>(n);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
      return *static_cast<const int16_t *>(n);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
      return *static_cast<const int8_t *>(n);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
      return *static_cast<const uint64_t *>(n);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
      return *static_cast<const uint32_t *>(n);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
      return *static_cast<const uint16_t *>(n);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
      return *static_cast<const uint8_t *>(n);
    default:
      throw std::invalid_argument("non-numeric member type_id");
  }
}

size_t total_member_offset(const MemberPath & path)
{
  size_t total = 0;
  for (const auto & member : path) {
    total += member->offset_;
  }
  return total;
}

MemberSequencePath assume_members_unindexed(const MemberPath & in_path)
{
  MemberSequencePath result;
  for (const auto & member : in_path) {
    auto & item = result.emplace_back();
    item.first = member;
    item.second = 0;
  }
  return result;
}

double get_numeric(const void * message, const MemberSequencePath & path, DataSourceOperator op)
{
  auto member_memory = static_cast<const uint8_t *>(message);
  uint8_t last_type;
  for (const auto & [member, idx] : path) {
    member_memory += member->offset_;
    if (member->is_array_) {
      auto next_mem = member->get_const_function(member_memory, idx);
      member_memory = static_cast<const uint8_t *>(next_mem);
    }
    last_type = member->type_id_;
  }
  if (op == DataSourceOperator::L2Norm) {
    auto vector = reinterpret_cast<const Vector3 *>(member_memory);
    return std::hypot(vector->x, vector->y, vector->z);
  }
  auto value = cast_numeric(member_memory, last_type);
  if (op == DataSourceOperator::Sqrt) {
    value = std::sqrt(value);
  }
  return value;
}

MemberSequencePathItemDescriptor to_descriptor_item(const MemberSequencePathItem & item)
{
  std::optional<size_t> idx = std::nullopt;
  if (item.first->is_array_) {
    idx = item.second;
  }
  return MemberSequencePathItemDescriptor {
    item.first->name_,
    idx,
  };
}

MemberSequencePathDescriptor to_descriptor(const MemberSequencePath & in_path)
{
  MemberSequencePathDescriptor out_path;
  std::transform(
    in_path.begin(), in_path.end(), std::back_inserter(out_path),
    &to_descriptor_item);
  return out_path;
}

bool contains_sequence(const MemberPath & member_path)
{
  for (const auto & member : member_path) {
    if (member->is_array_) {
      return true;
    }
  }
  return false;
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

MessageMemberContainer::MessageMemberContainer(
  const rosidl_message_type_support_t * introspection_support)
: introspection_support_(introspection_support)
{

}

MemberIterator MessageMemberContainer::begin() const
{
  return MemberIterator(introspection_support_);
}

MemberIterator MessageMemberContainer::end() const
{

  return MemberIterator(nullptr);
}

MessageIntrospection::MessageIntrospection(std::string message_type)
: message_type_(message_type)
{
  try {
    introspection_support_library_ = rclcpp::get_typesupport_library(
      message_type,
      rosidl_typesupport_introspection_cpp::typesupport_identifier);
  } catch (...) {
    // rclcpp throws if package cannot be found
    std::throw_with_nested(introspection_error("failed to load typesupport"));
  }
  if (!introspection_support_library_) {
    throw introspection_error("failed to load typesupport");
  }
  introspection_support_handle_ = rclcpp::get_typesupport_handle(
    message_type,
    rosidl_typesupport_introspection_cpp::typesupport_identifier,
    *introspection_support_library_);
  if (!introspection_support_handle_) {
    throw introspection_error("failed to load typesupport introspection_support_handle");
  }
}

const char * MessageIntrospection::message_type() const
{
  return message_type_.c_str();
}

const rosidl_message_type_support_t * MessageIntrospection::get_typesupport_handle() const
{
  return introspection_support_handle_;
}

std::optional<size_t> MessageIntrospection::get_header_offset() const
{
  auto m = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
    introspection_support_handle_->data);
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

MessageMemberContainer MessageIntrospection::members() const
{
  return MessageMemberContainer(introspection_support_handle_);
}

static bool matches(MemberPath path, const std::vector<std::string> & searched_member_path)
{
  auto it = path.cbegin();
  for (const auto & searched_member : searched_member_path) {
    if (it == path.cend()) {
      return false;
    }
    if (searched_member.compare((*it)->name_) != 0) {
      return false;
    }
    ++it;
  }
  return it == path.cend();
}

std::optional<MemberPath> MessageIntrospection::get_member_path(
  std::vector<std::string> member_path) const
{
  if (member_path.empty()) {
    std::invalid_argument("member_path required");
  }
  for (const auto & path : members()) {
    if (matches(path, member_path)) {
      return path;
    }
  }
  return std::nullopt;
}

std::optional<MemberSequencePath> MessageIntrospection::get_member_sequence_path(
  MemberSequencePathDescriptor in_path) const
{
  if (in_path.empty()) {
    std::invalid_argument("member_path required");
  }
  std::vector<std::string> names;
  for (const auto & [name, _] : in_path) {
    names.push_back(name);
  }
  auto member_path_opt = get_member_path(names);
  if (!member_path_opt.has_value()) {
    return std::nullopt;
  }
  auto member_path = member_path_opt.value();
  // member_path must be same size as in_path
  auto member_it = member_path.cbegin();

  MemberSequencePath result(in_path.size());
  MemberPath sub_path;
  for (size_t i = 0; i < result.size(); i++) {
    if (in_path[i].sequence_idx.has_value() && !(*member_it)->is_array_) {
      throw introspection_error(
              "member sequence path descriptor item defines index for non-array member");
    }
    result[i].first = *member_it;
    result[i].second = in_path[i].sequence_idx.value_or(0);
    ++member_it;
  }
  return result;
}

} // namespace quickplot

void write_member_sequence_path_item_descriptor(
  std::ostream & out,
  const quickplot::MemberSequencePathItemDescriptor & item)
{
  out << item.member_name;
  if (item.sequence_idx.has_value()) {
    out << "[" << item.sequence_idx.value() << "]";
  }
}

std::ostream & operator<<(std::ostream & out, const quickplot::MemberSequencePathDescriptor & path)
{
  for (auto it = path.cbegin(); it != path.cend(); ++it) {
    write_member_sequence_path_item_descriptor(out, *it);
    if (next(it) != path.cend()) {
      out << ".";
    }
  }
  return out;
}

std::ostream & operator<<(std::ostream & out, const quickplot::MemberPath & path)
{
  for (auto it = path.cbegin(); it != path.cend(); ++it) {
    out << (*it)->name_;
    if (next(it) != path.cend()) {
      out << ".";
    }
  }
  return out;
}

std::ostream & operator<<(std::ostream & out, const quickplot::MemberSequencePathItem & item)
{
  out << item.first->name_;
  if (item.first->is_array_) {
    out << "[" << item.second << "]";
  }
  return out;
}

std::ostream & operator<<(std::ostream & out, const quickplot::MemberSequencePath & path)
{
  for (auto it = path.cbegin(); it != path.cend(); ++it) {
    out << *it;
    if (next(it) != path.cend()) {
      out << ".";
    }
  }
  return out;
}
