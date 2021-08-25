#include <exception>
#include <iostream>
#include <string>

namespace quickplot
{
constexpr char TOPIC_NAME_PAYLOAD_LABEL[] = "quickplot_topic_name";
constexpr char FIELD_INFO_PAYLOAD_LABEL[] = "quickplot_field_info";

struct MemberPayload
{
  const char * topic_name;
  size_t member_index;
};

void print_exception_recursive(const std::exception & e, int level = 0, int after_level = 0)
{
  if (level >= after_level) {
    std::cerr << std::string(level, ' ') << e.what() << '\n';
  }
  try {
    std::rethrow_if_nested(e);
  } catch (const std::exception & e) {
    print_exception_recursive(e, level + 1, after_level);
  } catch (...) {
  }
}

} // namespace quickplot
