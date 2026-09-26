/**
 * @file log_record_handler.h
 * @brief ROS 2 adapter for Tesseract structured log records.
 */
#ifndef TESSERACT_ROSUTILS_LOG_RECORD_HANDLER_H
#define TESSERACT_ROSUTILS_LOG_RECORD_HANDLER_H

#include <tesseract/common/logging.h>

#include <string>

namespace tesseract_rosutils
{
/**
 * @brief Convert a native spdlog severity to its rcutils equivalent.
 * @param level Severity to convert.
 * @return The corresponding `RCUTILS_LOG_SEVERITY_*` value, or `RCUTILS_LOG_SEVERITY_UNSET` when disabled.
 */
int toRcutilsSeverity(spdlog::level::level_enum level) noexcept;

/**
 * @brief Render fields unsupported by rcutils as deterministic message text.
 * @param record Structured record to render.
 * @return The record message followed by its optional component name and attributes in deterministic key order.
 */
std::string formatRosMessage(const tesseract::common::LogRecord& record);

/** @brief Forward structured Tesseract records to ROS 2 logging while registered. */
class RosLogRecordHandler
{
public:
  /** @brief Construct a stopped handler. */
  RosLogRecordHandler() = default;

  /** @brief Stop forwarding records and release the registration. */
  ~RosLogRecordHandler();

  RosLogRecordHandler(const RosLogRecordHandler&) = delete;
  RosLogRecordHandler& operator=(const RosLogRecordHandler&) = delete;
  RosLogRecordHandler(RosLogRecordHandler&&) = delete;
  RosLogRecordHandler& operator=(RosLogRecordHandler&&) = delete;

  /**
   * @brief Begin forwarding structured records to rcutils.
   * @return True when the handler was newly registered; false when it was already started.
   */
  bool start();

  /** @brief Stop forwarding records. Calling this on a stopped handler has no effect. */
  void stop() noexcept;

  /** @return True when this handler is registered. */
  bool isStarted() const noexcept;

private:
  /** @brief Opaque registration identifier assigned by Tesseract common logging. */
  tesseract::common::LogRecordHandlerId handler_id_{ 0 };
};
}  // namespace tesseract_rosutils

#endif  // TESSERACT_ROSUTILS_LOG_RECORD_HANDLER_H