#include "implot.h" // NOLINT
#include <boost/algorithm/string/join.hpp>
#include <vector>
#include <mutex>
#include <string>
#include <memory>
#include <unordered_set>
#include <rclcpp/time.hpp>
#include "quickplot/config.hpp"
#include "quickplot/view_common.hpp"

namespace quickplot
{

struct PlotViewOptions
{
  bool use_sim_time;
  rclcpp::Time t_start;
  rclcpp::Time t_end;
  std::unordered_set<std::string> topics_with_time_reference_issues;
};

ImPlotPoint circular_buffer_access(void * data, int idx)
{
  auto buffer = reinterpret_cast<CircularBuffer::const_iterator *>(data);
  return buffer->operator[](idx);
}

bool PlotView(
  const char * id, std::shared_ptr<QuickPlotNode> node, PlotConfig & plot,
  PlotViewOptions options)
{
  for (ImPlotYAxis a = 0; a < static_cast<ImPlotYAxis>(plot.axes.size()); a++) {
    const auto & axis_config = plot.axes[a];
    ImPlot::SetNextPlotLimitsY(
      axis_config.y_min, axis_config.y_max,
      ImGuiCond_Once, a);
  }
  std::vector<double> history_tick_values_;
  std::vector<std::string> history_tick_labels_;

  size_t n_ticks = static_cast<size_t>((options.t_end - options.t_start).seconds());
  history_tick_values_.resize(n_ticks);
  if (n_ticks != history_tick_labels_.size()) {
    history_tick_labels_.resize(n_ticks);
    for (size_t i = 0; i < n_ticks; i++) {
      history_tick_labels_[i] =
        std::to_string(-static_cast<int>(n_ticks) + static_cast<int>(i));
    }
  }
  for (size_t i = 0; i < n_ticks; i++) {
    history_tick_values_[i] = options.t_end.seconds() - static_cast<int>(n_ticks) + i;
  }

  std::vector<char const *> tick_cstrings;
  tick_cstrings.reserve(history_tick_labels_.size());
  for (const auto & labels : history_tick_labels_) {
    tick_cstrings.push_back(const_cast<char *>(labels.c_str()));
  }

  auto start_sec = options.t_start.seconds();
  auto end_sec = options.t_end.seconds();
  ImPlot::SetNextPlotLimitsX(start_sec, end_sec, ImGuiCond_Always);
  ImPlot::SetNextPlotTicksX(
    history_tick_values_.data(),
    history_tick_values_.size(), tick_cstrings.data());

  std::stringstream x_label;
  x_label << "t (header.stamp sec)";
  if (options.use_sim_time) {
    x_label << " use_sim_time:=true";
  }

  std::unordered_set<std::string> time_warned_topics;
  if (ImPlot::BeginPlot(
      id, x_label.str().c_str(), nullptr, ImVec2(-1, -1),
      (plot.axes.size() >= 2 ? ImPlotFlags_YAxis2 : 0) |
      (plot.axes.size() >= 3 ? ImPlotFlags_YAxis3 : 0) | ImPlotFlags_NoTitle))
  {
    if (ImPlot::IsPlotXAxisHovered()) {
      ImGui::BeginTooltip();

      ImGui::Text("now: %.2f", options.t_end.seconds());
      ImGui::EndTooltip();
    }
    for (ImPlotYAxis a = 0; a < static_cast<ImPlotYAxis>(plot.axes.size()); a++) {
      for (auto source_it = plot.sources.begin(); source_it != plot.sources.end(); ) {
        bool removed = false;
        if (source_it->axis == a) {
          std::stringstream ss;
          auto resolved_topic = node->get_node_topics_interface()->resolve_topic_name(
            source_it->topic_name);
          ss << resolved_topic << " " << boost::algorithm::join(
            source_it->member_path, ".");

          ImPlot::SetPlotYAxis(a);

          std::unique_lock<std::mutex> lock(node->topic_mutex);
          auto it = node->topics_to_subscriptions.find(resolved_topic);
          std::string series_label;
          if (it == node->topics_to_subscriptions.end()) {
            // plot empty line to show warning in legend
            ss << " (not received)";
            series_label = ss.str();
            ImPlot::HideNextItem(true, ImGuiCond_Always);
            ImPlot::PlotLine(series_label.c_str(), static_cast<float *>(nullptr), 0);
          } else {
            auto & buffer = it->second.get_buffer(source_it->member_path);
            auto data = buffer.data();
            auto it = data.begin();
            series_label = ss.str();
            size_t item_count = data.size();
            ImPlot::PlotLineG(
              series_label.c_str(),
              &circular_buffer_access,
              &it,
              item_count);
            if (options.topics_with_time_reference_issues.find(resolved_topic) !=
              options.topics_with_time_reference_issues.end() &&
              time_warned_topics.find(resolved_topic) == time_warned_topics.end())
            {
              // Show warning about a probably time reference issue, since no data is visible.
              // TODO(ZeilingerM) Should be placed in the corner of the plot, like a toast
              //                  notification, but I haven't found an appropriate ImGui
              //                  feature to accomplish this within an ImPlot region.
              auto warning_color = ImPlot::GetLastItemColor();
              ImPlot::AnnotateClamped(
                (end_sec - start_sec) / 2.0, 0.5, ImVec2(
                  0,
                  0), warning_color, "WARNING [%s]: all data points are outside of the x range, it may be required to set use_sim_time when launching quickplot",
                source_it->topic_name.c_str());
              time_warned_topics.insert(resolved_topic);
            }
          }

          if (ImPlot::BeginLegendPopup(series_label.c_str())) {
            if (ImGui::Button("remove")) {
              source_it = plot.sources.erase(source_it);
              removed = true;
            }
            ImPlot::EndLegendPopup();
          }
        }
        if (!removed) {
          ++source_it;
        }
      }
    }
    for (ImPlotYAxis a = 0; a < static_cast<ImPlotYAxis>(plot.axes.size()); a++) {
      auto & axis_config = plot.axes[a];
      // store the potentially user-defined limit
      {
        auto limits = ImPlot::GetPlotLimits(a);
        axis_config.y_min = limits.Y.Min;
        axis_config.y_max = limits.Y.Max;
      }
    }
    return true;
  }
  return false;
}

void EndPlotView()
{
  ImPlot::EndPlot();
}

} // namespace quickplot
