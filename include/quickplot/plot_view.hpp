#include "implot.h" // NOLINT
#include <vector>
#include <mutex>
#include <string>
#include <memory>
#include <set>
#include <utility>
#include <unordered_set>
#include <boost/algorithm/string/join.hpp>
#include <rclcpp/time.hpp>
#include "quickplot/config.hpp"

namespace quickplot
{

enum class DataSourceError
{
  None = 0,
  // the message type behind the data source is not available on the system running the plot tool
  MessageTypeUnavailable,
  // the data source requests a member that is not part of the message type
  InvalidMember,
};

enum class DataWarning
{
  None = 0,
  // the timestamps in the last received messages were out of range
  TimeStampOutOfRange = 1,
};

struct SourceDescriptor
{
  DataSourceError error;
  std::vector<std::string> member_path;
};

struct ActiveDataSource
{
  DataWarning warning;
  // resolved introspection member path
  MemberPath member;
};

struct DataSource
{
  std::string id;
  std::string resolved_topic_name;
  std::variant<SourceDescriptor, ActiveDataSource> info;

  inline bool operator==(const DataSource & other) const
  {
    return id == other.id;
  }
};

} // namespace quickplot

namespace std
{
template<>
struct hash<quickplot::DataSource>
{
  inline size_t operator()(const quickplot::DataSource & source) const
  {
    return hash<std::string>().operator()(source.resolved_topic_name);
  }
};
} // namespace std

namespace quickplot
{

struct Plot
{
  // source and axis
  std::vector<std::pair<DataSource, int>> sources;
  std::vector<AxisConfig> axes;
};

struct PlotViewOptions
{
  bool use_sim_time;
  rclcpp::Time t_start;
  rclcpp::Time t_end;
};

ImPlotPoint circular_buffer_access(void * data, int idx)
{
  auto buffer = reinterpret_cast<CircularBuffer::const_iterator *>(data);
  return buffer->operator[](idx);
}

struct PlotViewResult
{
  bool displayed;
  float time_scale_delta;

  PlotViewResult()
  : displayed(false), time_scale_delta(0.0f) {}
};

PlotViewResult PlotView(const char * id, Plot & plot, const PlotViewOptions & options)
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

bool PlotSourcePopup(const DataSource & source)
{
  bool removed = false;
  if (ImPlot::BeginLegendPopup(source.id.c_str())) {
    if (ImGui::Button("remove")) {
      removed = true;
    }
    ImPlot::EndLegendPopup();
  }
  return removed;
}

void PlotSource(const DataSource & source, const PlotDataContainer & data)
{
  ImPlot::HideNextItem(false, ImGuiCond_Always);
  auto it = data.begin();
  size_t item_count = data.size();
  ImPlot::PlotLineG(
    source.id.c_str(),
    &circular_buffer_access,
    &it,
    item_count);
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

void PlotSourceError(
  const DataSource & source, const char * message,
  const PlotViewOptions & plot_opts)
{
  // plot empty line to show the item in legend, even though it is not received yet
  // this allows the user to remove the source from the list
  ImPlot::HideNextItem(true, ImGuiCond_Always);
  ImPlot::PlotLine(source.id.c_str(), static_cast<float *>(nullptr), 0);
  auto warning_color = ImPlot::GetLastItemColor();

  auto start_sec = plot_opts.t_start.seconds();
  auto end_sec = plot_opts.t_end.seconds();

  auto warn_x_pos = (end_sec - start_sec) / 2.0;
  auto warn_y_pos = 0.5;
  auto warn_offset = ImVec2(0, 0);

  ImPlot::AnnotateClamped(
    warn_x_pos, warn_y_pos, warn_offset, warning_color, "[%s]: %s",
    source.resolved_topic_name.c_str(), message);
}

void EndPlotView()
{
  ImPlot::EndPlot();
}

} // namespace quickplot
