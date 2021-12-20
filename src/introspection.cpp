#include <vector>
#include <stdexcept>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <boost/utility.hpp> // for next(it)
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>
#include "quickplot/introspection.hpp"

namespace quickplot
{

std::ostream & operator<<(std::ostream & out, const MemberPtr & member)
{
  out << member->name_;
  return out;
}

std::string fmt_member_path(const MemberPath & member)
{
  std::stringstream ss;
  for (auto it = member.cbegin(); it != member.cend(); ++it) {
    ss << (*it)->name_;
    if (next(it) != member.cend()) {
      ss << ".";
    }
  }
  return ss.str();
}

std::vector<std::string> member_path_as_strvec(const MemberPath & path)
{
  std::vector<std::string> result;
  for (const auto & member : path) {
    result.push_back(member->name_);
  }
  return result;
}

std::string source_id(const std::string & topic, const std::vector<std::string> & members)
{
  std::stringstream ss;
  ss << topic << "/" << boost::algorithm::join(members, ".");
  return ss.str();
}

std::string source_id(const std::string & topic, const MemberPath & member_path)
{
  std::stringstream ss;
  ss << topic << "/" << fmt_member_path(member_path);
  return ss.str();
}

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
      throw std::invalid_argument("non-numeric member type_id");
  }
}

size_t total_member_offset(MemberPath path)
{
  size_t total = 0;
  for (const auto & member : path) {
    total += member->offset_;
  }
  return total;
}

double get_numeric(void * message, MemberPath member)
{
  auto bytes = reinterpret_cast<uint8_t *>(message) + total_member_offset(member);
  return cast_numeric(bytes, member.back()->type_id_);
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
  for (const auto& searched_member : searched_member_path) {
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

std::optional<MemberPath> MessageIntrospection::get_member(
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

} // namespace quickplot
