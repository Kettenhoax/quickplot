#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <cmath>
#include <string>
#include <vector>
#include <memory>
#include <deque>

namespace quickplot
{

struct MessageMemberInfo
{
  // offset into memory of the entire message to obtain member
  uint32_t offset;
  uint8_t type_id;
};

struct MessageMember
{
  // path of member in message tree
  // for example, the path of linear.x in geometry_msgs/Twist is {"linear", "x"}
  std::vector<std::string> path;
  MessageMemberInfo info;
};

class MemberIterator : public std::iterator<std::input_iterator_tag, MessageMember>
{
private:
  struct MemberIteration
  {
    const rosidl_message_type_support_t * message;
    size_t offset;
    size_t current_index;
  };

  const rosidl_typesupport_introspection_cpp::MessageMember * get_member(
    const MemberIteration & it)
  {
    auto members =
      static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(it.message->data);
    return &members->members_[it.current_index];
  }

  void _advance_to_leaf()
  {
    while (true) {
      // ensure current iteration is a leaf item
      const auto it = deque_.back();
      auto member = get_member(it);
      if (member->type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
        deque_.push_back(
          MemberIteration {
            .message = member->members_,
            .offset = it.offset + member->offset_,
            .current_index = 0,
          });
      } else {
        break;
      }
    }
  }

  void _set_current_member()
  {
    current_member_.path.clear();
    for (const auto& it : deque_) {
      auto super_member = get_member(it);
      current_member_.path.push_back(super_member->name_);
    }
    auto leaf = deque_.back();
    auto member = get_member(leaf);
    current_member_.info.type_id = member->type_id_;
    current_member_.info.offset = leaf.offset + member->offset_;
  }

public:
  explicit MemberIterator(const rosidl_message_type_support_t * introspection_support)
  {
    if (introspection_support) {
      deque_.push_back(
        MemberIteration {
          .message = introspection_support,
          .offset = 0,
          .current_index = 0,
        });
      _advance_to_leaf();
      if (!deque_.empty()) {
        _set_current_member();
      }
    }
  }

  bool operator!=(const MemberIterator & rhs) const
  {
    if (deque_.size() != rhs.deque_.size()) {
      return true;
    }
    for (size_t i = 0; i < deque_.size(); i++) {
      if (deque_[i].message != rhs.deque_[i].message) {
        return true;
      }
      if (deque_[i].current_index != rhs.deque_[i].current_index) {
        return true;
      }
    }
    return false;
  }

  MessageMember & operator++()
  {
    assert(!deque_.empty());
    while (!deque_.empty()) {
      auto & it = deque_.back();
      it.current_index += 1;
      auto members =
        static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(it.message->data);
      if (it.current_index >= members->member_count_) {
        deque_.pop_back();
      } else {
        _advance_to_leaf();
        break;
      }
    }
    if (!deque_.empty()) {
      _set_current_member();
    }
    return current_member_;
  }

  MessageMember operator++(int) {assert(false);}

  MessageMember operator*() const
  {
    return current_member_;
  }

  MessageMember & operator*()
  {
    return current_member_;
  }

  MessageMember const * operator->() const
  {
    return &current_member_;
  }

private:
  std::deque<MemberIteration> deque_;
  MessageMember current_member_;
};

class MessageIntrospection
{

private:
  std::string message_type_;
  std::shared_ptr<rcpputils::SharedLibrary> introspection_support_library_;
  const rosidl_message_type_support_t * introspection_support_handle_;

public:
  explicit MessageIntrospection(std::string message_type);

  // disable copy and move
  MessageIntrospection & operator=(MessageIntrospection && other) = delete;

  std::string message_type() const;

  const rosidl_typesupport_introspection_cpp::MessageMembers * members() const;

  std::optional<size_t> get_header_offset() const;

  MemberIterator begin_member_infos() const;

  MemberIterator end_member_infos() const;

  std::optional<MessageMemberInfo> get_member_info(std::vector<std::string> member_path) const;
};

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

  std::string message_type() const;

  // disable copy and move
  IntrospectionMessageDeserializer & operator=(IntrospectionMessageDeserializer && other) = delete;

  std::vector<uint8_t> init_buffer() const;

  void fini_buffer(std::vector<uint8_t> & buffer) const;

  void deserialize(const rclcpp::SerializedMessage &, void * message) const;

  rclcpp::Time get_header_stamp(void * message) const;

  double get_numeric(void * message, MessageMemberInfo info) const;
};

} // namespace quickplot
