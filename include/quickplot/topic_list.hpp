#pragma once

#include <set>
#include <utility>
#include <memory>
#include <vector>
#include <string>
#include <rcpputils/asserts.hpp>
#include "quickplot/resources.hpp"
#include "quickplot/plot_subscription.hpp"

namespace quickplot
{

constexpr const char * TOPIC_LIST_WINDOW_ID = "TopicList";

// payload describing which topic entry has been clicked to add a new plot
// nullopt if none has been clicked this frame
using ClickPayload = std::optional<MemberPayload>;

using RenderResultWithPayload = std::pair<bool, ClickPayload>;

bool TopicEntryError(const std::string & topic, const MessageTypeError & error)
{
  ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyle().Colors[ImGuiCol_TextDisabled]);
  if (ImGui::TreeNodeEx(topic.c_str(), ImGuiTreeNodeFlags_Leaf)) {
    ImGui::PopStyleColor();
    // color was eye-dropped from Rviz display warnings
    // TODO(ZeilingerM) single hardcoded color does not work well with theme changes
    static ImVec4 WARNING_COLOR = static_cast<ImVec4>(ImColor::HSV(0.1083f, 0.968f, 0.867f));
    ImGui::PushStyleColor(ImGuiCol_Text, WARNING_COLOR);
    ImGui::TextWrapped("message type '%s' is not available:", error.message_type.c_str());
    ImGui::TextWrapped("%s", error.error_message.c_str());
    ImGui::PopStyleColor();
    return true;
  }
  ImGui::PopStyleColor();
  return false;
}

RenderResultWithPayload TopicEntryInitialized(
  const std::string & topic,
  const std::shared_ptr<MessageIntrospection> & introspection)
{
  ClickPayload click_payload = std::nullopt;
  if (ImGui::TreeNode(topic.c_str())) {
    if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_SourceAllowNullID)) {
      ImGui::SetDragDropPayload("topic_name", topic.c_str(), topic.size());
      ImPlot::ItemIcon(ImGui::GetColorU32(ImGuiCol_Text));
      ImGui::SameLine();
      ImGui::Text("Dragging %s", topic.c_str());
      ImGui::EndDragDropSource();
    }
    size_t i {0};
    for (const auto & member : introspection->members()) {
      if (is_numeric(member.back()->type_id_) && !contains_sequence(member)) {
        std::stringstream ss;
        ss << member;
        auto member_str = ss.str();
        ImGui::Selectable(member_str.c_str(), false);
        MemberPayload payload {
          .topic_name = topic.c_str(),
          .member = assume_members_unindexed(member),
        };
        if (ImGui::IsItemHovered()) {
          ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);

          if (GImGui->HoveredIdTimer > 0.5) {
            ImGui::BeginTooltip();
            ImGui::Text("add %s to plot0", member_str.c_str());
            ImGui::Text("or drag and drop on plot of choice");
            ImGui::EndTooltip();
          }
          if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
            click_payload = payload;
          }
        }
        if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_SourceAllowNullID)) {
          ImGui::SetDragDropPayload("topic_member", &payload, sizeof(payload));
          ImGui::EndDragDropSource();
        }
      }
      ++i;
    }
    return {true, click_payload};
  }
  return {false, click_payload};
}

RenderResultWithPayload TopicEntry(const std::string & topic, const MessageTypeInfo & type_info)
{
  return std::visit(
    overloaded {
      [topic](const MessageIntrospectionPtr & introspection) {
        return TopicEntryInitialized(topic, introspection);
      },
      [topic](const MessageTypeError & error) {
        ClickPayload empty_payload = std::nullopt;
        return std::make_pair(TopicEntryError(topic, error), empty_payload);
      }
    }, type_info);
}

void EndTopicEntry()
{
  ImGui::TreePop();
}

ClickPayload TopicList(
  const TopicTypeMap & topics_to_types,
  std::vector<Plot> & plots,
  std::shared_ptr<QuickPlotNode> node)
{
  ClickPayload payload;

  ImGuiWindowFlags list_window_flags = ImGuiWindowFlags_None;
  if (ImGui::Begin(TOPIC_LIST_WINDOW_ID, nullptr, list_window_flags)) {
    // accumulate and sort subscriptions of active plots
    auto cmp_topic_names =
      [](std::shared_ptr<PlotSubscription> s1, std::shared_ptr<PlotSubscription> s2) {
        return s1->topic_name() < s2->topic_name();
      };
    std::set<std::shared_ptr<PlotSubscription>, decltype(cmp_topic_names)> active_topics(
      cmp_topic_names);
    for (auto & plot : plots) {
      for (auto & [series, _] : plot.series) {
        auto active = std::get_if<ActiveDataSource>(&series.source);
        if (active) {
          active_topics.insert(active->subscription);
        }
      }
    }

    // display list of subscribed topics and their receive stats
    if (!active_topics.empty()) {
      if (ImGui::CollapsingHeader("active topics", ImGuiTreeNodeFlags_DefaultOpen)) {
        for (const auto & subscription : active_topics) {
          auto type_it = topics_to_types.find(subscription->topic_name());
          rcpputils::assert_true(
            type_it != topics_to_types.end(),
            "topics can only become active when their type is known");
          const auto & [render_result, _] = TopicEntry(
            subscription->topic_name(), type_it->second);
          if (render_result) {
            auto stats = subscription->receive_period_stats();
            if (stats.standard_deviation < (stats.average / 10.0) && stats.average < 1.0 &&
              stats.average > 0.001)
            {
              ImGui::Text("%.1f hz", 1.0 / stats.average);
            }
            EndTopicEntry();
          }
        }
      }
    }

    // display filtered list of topics; include those that can't be subscribed with a warning
    if (ImGui::CollapsingHeader("available topics", ImGuiTreeNodeFlags_DefaultOpen)) {
      // according to https://design.ros2.org/articles/topic_and_service_names.html, max topic
      // name length is 256
      const size_t filter_text_length = 256 + 1;
      static char filter_text_raw[filter_text_length];
      ImGui::PushItemWidth(-1);
      ImGui::InputTextWithHint("", "filter topic or type", filter_text_raw, filter_text_length);
      ImGui::PopItemWidth();

      std::string filter_text(filter_text_raw);

      std::vector<TopicTypeMap::const_iterator> shown_available_topics;
      size_t total_available = 0;
      auto it = topics_to_types.begin();
      for (; it != topics_to_types.end(); ++it) {
        const auto & [topic, type] = *it;
        if (node->is_subscribed_to(topic)) {
          // skip subscribed topics, those are already listed in the 'active topics' section
          continue;
        }
        ++total_available;
        if (topic.find(filter_text) == std::string::npos) {
          continue;
        }
        shown_available_topics.push_back(it);
      }

      // defer rendering the items up to this point, to print the number of shown topics before the topics
      if (filter_text[0] != '\0') {
        // if filter is passed, show amount of filtered topics
        ImGui::PushItemWidth(-1);
        ImGui::Text("Showing %lu of %lu topics", shown_available_topics.size(), total_available);
        ImGui::PopItemWidth();
      }
      for (const auto & it : shown_available_topics) {
        const auto& [render_result, entry_payload] = TopicEntry(it->first, it->second);
        if (render_result) {
          payload = entry_payload;
          EndTopicEntry();
        }
      }
    }
  }
  ImGui::End();
  return payload;
}

} // namespace quickplot
