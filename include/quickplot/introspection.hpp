#pragma once
#include <deque>
#include <list>
#include <vector>
#include <string>
#include <sstream>
#include <memory>
#include <assert.h>
#include <optional>
#include <utility>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <rcpputils/shared_library.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>

namespace quickplot
{

struct introspection_error : public std::exception
{
  std::string message_;

  explicit introspection_error(std::string message)
  : message_(message)
  {

  }

  const char * what() const throw ()
  {
    return message_.c_str();
  }
};

enum class DataSourceOperator {
  Identity,
  Sqrt,
};

struct MemberSequencePathItemDescriptor
{
  std::string member_name;
  std::optional<size_t> sequence_idx;

  inline bool operator==(const MemberSequencePathItemDescriptor & other) const
  {
    return member_name == other.member_name && sequence_idx == other.sequence_idx;
  }
};

// sequence of member names and indices, to find as members in introspection typesupport
using MemberSequencePathDescriptor = std::vector<MemberSequencePathItemDescriptor>;

using MemberPtr = const rosidl_typesupport_introspection_cpp::MessageMember *;

// sequence of introspection typesupport members within a message, from outer member to inner
using MemberPath = std::list<MemberPtr>;

// introspection typesupport member and its index to access a specific item, if the member is a
// sequence
using MemberSequencePathItem = std::pair<MemberPtr, size_t>;

using MemberSequencePath = std::vector<MemberSequencePathItem>;

struct MessageAccessor {
  // resolved member path of topic type
  MemberSequencePath member;

  // operator to apply to the member
  DataSourceOperator op;
};

double cast_numeric(const void * n, uint8_t type_id);

bool is_numeric(uint8_t type_id);

bool contains_sequence(const MemberPath &);

size_t total_member_offset(const MemberPath &);

MemberSequencePath assume_members_unindexed(const MemberPath &);

double get_numeric(const void *, const MemberSequencePath &, DataSourceOperator = DataSourceOperator::Identity);

MemberSequencePathItemDescriptor to_descriptor_item(const MemberSequencePathItem &);

MemberSequencePathDescriptor to_descriptor(const MemberSequencePath &);

class MemberIterator : public std::iterator<std::forward_iterator_tag, MemberPath>
{
private:
  struct MemberIteration
  {
    const rosidl_typesupport_introspection_cpp::MessageMembers * members;
    // index into the selected message member
    size_t index;

    const rosidl_typesupport_introspection_cpp::MessageMember * member()
    {
      return &members->members_[index];
    }

    bool is_at_end()
    {
      return index + 1 >= members->member_count_;
    }

    bool is_past_end()
    {
      return index >= members->member_count_;
    }
  };

  std::deque<MemberIteration> deque_;
  MemberPath value_;

  void _advance()
  {
    assert(!deque_.empty());

    // case 1: expand one level of nested message
    auto curr = deque_.back().member();
    if (curr->type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
      auto & next_it = deque_.emplace_back();
      next_it.members =
        static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(curr
        ->members_->data);
      next_it.index = 0;
      // append new expanded member to member path
      value_.push_back(deque_.back().member());
      return;
    }

    // case 2: we're at the end of a nested member, and need to backtrack to the next sibling
    while (!deque_.empty() && deque_.back().is_at_end()) {
      value_.pop_back();
      deque_.pop_back();
      if (deque_.empty()) {
        return;
      }
      deque_.back().index++;
      if (!deque_.back().is_past_end()) {
        // move to next-up available member in message
        value_.pop_back();
        value_.push_back(deque_.back().member());
        return;
      }
    }

    if (deque_.empty()) {
      // case 2 could advance to the end of the iterator
      return;
    }

    // case 3: increment member in current message
    deque_.back().index++;
    curr = deque_.back().member();
    value_.pop_back();
    value_.push_back(curr);
  }

public:
  explicit MemberIterator(const rosidl_message_type_support_t * introspection_support)
  {
    if (introspection_support) {
      deque_.push_back(
        MemberIteration {
          .members =
          static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
            introspection_support
            ->data),
          .index = 0,
        });
      value_.push_back(deque_.back().members->members_);
    }
  }

  bool operator!=(const MemberIterator & rhs) const
  {
    if (deque_.size() != rhs.deque_.size()) {
      return true;
    }
    for (size_t i = 0; i < deque_.size(); i++) {
      if (deque_[i].members != rhs.deque_[i].members) {
        return true;
      }
      if (deque_[i].index != rhs.deque_[i].index) {
        return true;
      }
    }
    return false;
  }

  MemberPath & operator++()
  {
    assert(!deque_.empty());
    _advance();
    return value_;
  }

  MemberPath operator++(int) {assert(false);}

  MemberPath operator*() const
  {
    assert(!value_.empty());
    return value_;
  }

  MemberPath & operator*()
  {
    assert(!value_.empty());
    return value_;
  }

  MemberPath const * operator->() const
  {
    assert(!value_.empty());
    return &value_;
  }
};

class MessageMemberContainer
{
private:
  const rosidl_message_type_support_t * introspection_support_;

public:
  explicit MessageMemberContainer(const rosidl_message_type_support_t *);

  MemberIterator begin() const;

  MemberIterator end() const;
};

class MessageIntrospection
{
private:
  std::string message_type_;
  std::shared_ptr<rcpputils::SharedLibrary> introspection_support_library_;
  const rosidl_message_type_support_t * introspection_support_handle_;

public:
  explicit MessageIntrospection(std::string message_type);

  const char * message_type() const;

  const rosidl_message_type_support_t * get_typesupport_handle() const;

  // disable copy and move
  MessageIntrospection & operator=(MessageIntrospection && other) = delete;

  MessageMemberContainer members() const;

  std::optional<size_t> get_header_offset() const;

  std::optional<MemberPath> get_member_path(std::vector<std::string> member_path) const;

  std::optional<MemberSequencePath> get_member_sequence_path(
    MemberSequencePathDescriptor in_path) const;
};

} // namespace quickplot

// TODO(ZeilingerM) couldn't figure out how to compute the ostream overload (gcc 9.3)
// std::ostream & operator<<(std::ostream &, const quickplot::MemberSequencePathItemDescriptor &);
void write_member_sequence_path_item_descriptor(
  std::ostream &,
  const quickplot::MemberSequencePathItemDescriptor &);

std::ostream & operator<<(std::ostream &, const quickplot::MemberSequencePathDescriptor &);

std::ostream & operator<<(std::ostream &, const quickplot::MemberPath &);

std::ostream & operator<<(std::ostream &, const quickplot::MemberSequencePathItem &);

std::ostream & operator<<(std::ostream &, const quickplot::MemberSequencePath &);
