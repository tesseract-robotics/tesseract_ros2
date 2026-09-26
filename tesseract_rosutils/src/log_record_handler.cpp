#include <tesseract_rosutils/log_record_handler.h>

#include <rcutils/logging.h>

#include <algorithm>
#include <sstream>
#include <type_traits>
#include <utility>
#include <vector>

namespace tesseract_rosutils
{
namespace
{
std::string formatAttribute(const tesseract::common::LogAttribute& attribute)
{
  return std::visit(
      [](const auto& value) {
        using ValueType = std::decay_t<decltype(value)>;
        if constexpr (std::is_same_v<ValueType, bool>)
          return value ? std::string("true") : std::string("false");
        else if constexpr (std::is_same_v<ValueType, std::string>)
          return value;
        else
          return std::to_string(value);
      },
      attribute);
}

void appendField(std::ostringstream& stream, const char* name, const std::string& value)
{
  if (!value.empty())
    stream << ' ' << name << '=' << value;
}

void forwardRecord(const tesseract::common::LogRecord& record) noexcept
{
  try
  {
    const int severity = toRcutilsSeverity(record.level);
    if (severity == RCUTILS_LOG_SEVERITY_UNSET)
      return;

    const char* logger_name = record.logger_name.empty() ? "tesseract" : record.logger_name.c_str();
    RCUTILS_LOGGING_AUTOINIT;
    if (!rcutils_logging_logger_is_enabled_for(logger_name, severity))
      return;

    const std::size_t line =
        record.source_location.line > 0 ? static_cast<std::size_t>(record.source_location.line) : 0;
    const rcutils_log_location_t location{ record.source_location.funcname, record.source_location.filename, line };
    const std::string message = formatRosMessage(record);
    rcutils_log(&location, severity, logger_name, "%s", message.c_str());
  }
  catch (...)
  {
    return;
  }
}
}  // namespace

int toRcutilsSeverity(spdlog::level::level_enum level) noexcept
{
  switch (level)
  {
    case spdlog::level::trace:
    case spdlog::level::debug:
      return RCUTILS_LOG_SEVERITY_DEBUG;
    case spdlog::level::info:
      return RCUTILS_LOG_SEVERITY_INFO;
    case spdlog::level::warn:
      return RCUTILS_LOG_SEVERITY_WARN;
    case spdlog::level::err:
      return RCUTILS_LOG_SEVERITY_ERROR;
    case spdlog::level::critical:
      return RCUTILS_LOG_SEVERITY_FATAL;
    case spdlog::level::off:
    case spdlog::level::n_levels:
      return RCUTILS_LOG_SEVERITY_UNSET;
  }
  return RCUTILS_LOG_SEVERITY_UNSET;
}

std::string formatRosMessage(const tesseract::common::LogRecord& record)
{
  std::ostringstream stream;
  stream << record.message;
  appendField(stream, "component_name", record.component_name);

  std::vector<std::pair<std::string, std::string>> attributes;
  attributes.reserve(record.attributes.size());
  for (const auto& [name, value] : record.attributes)
    attributes.emplace_back(name, formatAttribute(value));
  std::sort(attributes.begin(), attributes.end());
  for (const auto& [name, value] : attributes)
    stream << ' ' << name << '=' << value;

  return stream.str();
}

RosLogRecordHandler::~RosLogRecordHandler() { stop(); }

bool RosLogRecordHandler::start()
{
  if (isStarted())
    return false;

  handler_id_ =
      tesseract::common::addLogRecordHandler([](const tesseract::common::LogRecord& record) { forwardRecord(record); });
  return isStarted();
}

void RosLogRecordHandler::stop() noexcept
{
  if (!isStarted())
    return;

  tesseract::common::removeLogRecordHandler(handler_id_);
  handler_id_ = 0;
}

bool RosLogRecordHandler::isStarted() const noexcept { return handler_id_ != 0; }
}  // namespace tesseract_rosutils