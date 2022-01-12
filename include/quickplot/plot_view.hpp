#pragma once

#include "implot.h" // NOLINT
#include <mutex>
#include <string>
#include <vector>
#include <set>
#include <unordered_set>
#include <boost/algorithm/string/join.hpp>
#include <rclcpp/time.hpp>
#include "quickplot/plot.hpp"

namespace quickplot
{

struct PlotViewOptions
{
  bool use_sim_time;
  rclcpp::Time t_start;
  rclcpp::Time t_end;
};

ImPlotPoint circular_buffer_get_item(void * data, int idx)
{
  auto buffer = static_cast<CircularBuffer::const_iterator *>(data);
  return buffer->operator[](idx);
}

struct PlotViewResult
{
  bool displayed;
  float time_scale_delta;

  PlotViewResult()
  : displayed(false), time_scale_delta(0.0f) {}
};

PlotViewResult BeginPlotView(const char * id, Plot & plot, const PlotViewOptions & options)
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
  PlotViewResult result;
  if (ImPlot::BeginPlot(
      id, x_label.str().c_str(), nullptr, ImVec2(-1, -1),
      (plot.axes.size() >= 2 ? ImPlotFlags_YAxis2 : 0) |
      (plot.axes.size() >= 3 ? ImPlotFlags_YAxis3 : 0) | ImPlotFlags_NoTitle))
  {
    if (ImPlot::IsPlotXAxisHovered()) {
      ImGui::BeginTooltip();
      ImGui::Text("now: %.2f", options.t_end.seconds());
      ImGui::EndTooltip();

      result.time_scale_delta = ImGui::GetIO().MouseWheel;
    }
    result.displayed = true;
  }
  return result;
}

bool PlotSeriesPopup(const std::string & id)
{
  bool removed = false;
  if (ImPlot::BeginLegendPopup(id.c_str())) {
    if (ImGui::Button("remove")) {
      removed = true;
    }
    ImPlot::EndLegendPopup();
  }
  return removed;
}

void PlotSource(const std::string & id, const ActiveDataSource & source)
{
  auto data = source.data->data();
  auto it = data->begin();
  ImPlot::PlotLineG(
    id.c_str(),
    &circular_buffer_get_item,
    &it,
    data->size());
}

void PlotSourceStddev(
  const std::string & id, const ActiveDataSource & source,
  const ActiveDataSource & stddev)
{
  auto stddev_data = stddev.data->data();
  auto source_data = source.data->data();

  std::vector<double> times(source_data->size());
  std::vector<double> lower(source_data->size());
  std::vector<double> upper(source_data->size());

  auto stddev_vals = sync_right(
    source_data->begin(), source_data->end(),
    stddev_data->begin(), stddev_data->end(), 0.0);
  assert(stddev_vals.size() == source_data->size());

  auto source_it = source_data->begin();
  for (size_t i = 0; i < lower.size(); i++) {
    ImPlotPoint data_val = source_it[i];
    times[i] = data_val.x;
    lower[i] = data_val.y - stddev_vals[i];
    upper[i] = data_val.y + stddev_vals[i];
  }

  ImPlot::PushStyleVar(ImPlotStyleVar_FillAlpha, 0.25f);
  ImPlot::PlotShaded(id.c_str(), times.data(), lower.data(), upper.data(), times.size());
  ImPlot::PopStyleVar();
}

static const char * UninitializedText = "data not received yet";
static const char * MessageTypeUnavailableText = "failed to load message type";
static const char * InvalidMemberText = "invalid message member";
static const char * TimeStampOutOfRangeText =
  "all samples are outside of the x range; if you are using sim time set use_sim_time:=true when launching quickplot";

const char * get_error_message(DataSourceError error)
{
  if (error == DataSourceError::None) {
    return UninitializedText;
  }
  if (error == DataSourceError::MessageTypeUnavailable) {
    return MessageTypeUnavailableText;
  }
  if (error == DataSourceError::InvalidMember) {
    return InvalidMemberText;
  }
  return nullptr;
}

const char * get_warning_message(DataWarning warning)
{
  if (warning == DataWarning::TimeStampOutOfRange) {
    return TimeStampOutOfRangeText;
  }
  return nullptr;
}

void PlotSeriesError(
  const TimeSeries & series, const char * message,
  const PlotViewOptions & plot_opts)
{
  // plot empty line to show the item in legend, even though it is not received yet
  // this allows the user to remove the source from the list
  ImPlot::HideNextItem(true, ImGuiCond_Always);
  ImPlot::PlotLine(series.id.c_str(), static_cast<float *>(nullptr), 0);
  auto warning_color = ImPlot::GetLastItemColor();

  auto start_sec = plot_opts.t_start.seconds();
  auto end_sec = plot_opts.t_end.seconds();

  auto warn_x_pos = (end_sec - start_sec) / 2.0;
  auto warn_y_pos = 0.5;
  auto warn_offset = ImVec2(0, 0);

  ImPlot::AnnotateClamped(
    warn_x_pos, warn_y_pos, warn_offset, warning_color, "[%s]: %s",
    series.topic_name().c_str(), message);
}

std::optional<SeriesPayload> PlotView(Plot & plot, const PlotViewOptions & plot_opts)
{
  std::optional<SeriesPayload> result;
  for (ImPlotYAxis a = 0; a < static_cast<ImPlotYAxis>(plot.axes.size()); a++) {
    for (auto series_it = plot.series.begin(); series_it != plot.series.end(); ) {
      auto axis = series_it->second;
      if (axis != a) {
        // ensure series occurs in this plot axis
        ++series_it;
        continue;
      }
      const auto & series = series_it->first;

      // set axis just before matching source was found, to ensure axis is only displayed
      // if corresponding source exists
      ImPlot::SetPlotYAxis(a);

      // display either data or detected errors related to the data
      std::visit(
        overloaded {
          [series, &plot_opts](const ActiveDataSource & active) {
            if (active.warning == DataWarning::None) {
              ImPlot::HideNextItem(false, ImGuiCond_Always);
              PlotSource(series.id, active);

              auto stddev_active = std::get_if<ActiveDataSource>(&series.stddev_source);
              if (stddev_active) {
                PlotSourceStddev(series.id, active, *stddev_active);
              }
            } else {
              PlotSeriesError(series, get_warning_message(active.warning), plot_opts);
            }
          },
          [series, &plot_opts](const SourceInfo & descriptor) {
            PlotSeriesError(series, get_error_message(descriptor.error), plot_opts);
          }
        }, series.source);

      if (PlotSeriesPopup(series.id)) {
        series_it = plot.series.erase(series_it);
      } else {
        ++series_it;
      }
    }
    auto & axis_config = plot.axes[a];
    // store the current plot limits, in case they were changed via user input
    auto limits = ImPlot::GetPlotLimits(a);
    axis_config.y_min = limits.Y.Min;
    axis_config.y_max = limits.Y.Max;
  }

  if (ImPlot::BeginDragDropTarget()) {
    if (const ImGuiPayload * payload = ImGui::AcceptDragDropPayload("topic_member")) {
      result = SeriesPayload {
        .member = *static_cast<MemberPayload *>(payload->Data),
        .axis = ImPlotYAxis_1,
      };
    }
    ImPlot::EndDragDropTarget();
  }
  for (auto axis : {ImPlotYAxis_1, ImPlotYAxis_2}) {
    if (ImPlot::BeginDragDropTargetY(axis)) {
      if (const ImGuiPayload * payload = ImGui::AcceptDragDropPayload("topic_member")) {
        result = SeriesPayload {
          .member = *static_cast<MemberPayload *>(payload->Data),
          .axis = axis,
        };
      }
      ImPlot::EndDragDropTarget();
    }
  }
  return result;
}

void EndPlotView()
{
  ImPlot::EndPlot();
}

} // namespace quickplot
