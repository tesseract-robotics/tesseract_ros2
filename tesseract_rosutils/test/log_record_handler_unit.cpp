#include <tesseract_rosutils/log_record_handler.h>

#include <gtest/gtest.h>
#include <rcutils/logging.h>

#include <array>
#include <cstdarg>
#include <cstdio>
#include <string>

namespace
{
struct CapturedLog
{
  int severity{ RCUTILS_LOG_SEVERITY_UNSET };
  std::string name;
  std::string function;
  std::string file;
  std::size_t line{ 0 };
  std::string message;
};

CapturedLog& getCapturedLog()
{
  static CapturedLog captured_log;
  return captured_log;
}

void captureOutput(const rcutils_log_location_t* location,
                   int severity,
                   const char* name,
                   rcutils_time_point_value_t,
                   const char* format,
                   va_list* args)
{
  std::array<char, 1024> message{};
  va_list args_copy;
  va_copy(args_copy, *args);
  std::vsnprintf(message.data(), message.size(), format, args_copy);
  va_end(args_copy);

  CapturedLog& captured_log = getCapturedLog();
  captured_log.severity = severity;
  captured_log.name = name == nullptr ? "" : name;
  captured_log.function = location == nullptr || location->function_name == nullptr ? "" : location->function_name;
  captured_log.file = location == nullptr || location->file_name == nullptr ? "" : location->file_name;
  captured_log.line = location == nullptr ? 0 : location->line_number;
  captured_log.message = message.data();
}

class RosLogRecordHandlerUnit : public testing::Test
{
protected:
  void SetUp() override
  {
    previous_handler_ = rcutils_logging_get_output_handler();
    previous_level_ = rcutils_logging_get_default_logger_level();
    rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_DEBUG);
    rcutils_logging_set_output_handler(captureOutput);
    getCapturedLog() = {};
  }

  void TearDown() override
  {
    rcutils_logging_set_output_handler(previous_handler_);
    rcutils_logging_set_default_logger_level(previous_level_);
  }

  rcutils_logging_output_handler_t previous_handler_{ nullptr };
  int previous_level_{ RCUTILS_LOG_SEVERITY_INFO };
};
}  // namespace

TEST_F(RosLogRecordHandlerUnit, MapsSeverity)
{
  EXPECT_EQ(tesseract_rosutils::toRcutilsSeverity(spdlog::level::trace), RCUTILS_LOG_SEVERITY_DEBUG);
  EXPECT_EQ(tesseract_rosutils::toRcutilsSeverity(spdlog::level::warn), RCUTILS_LOG_SEVERITY_WARN);
  EXPECT_EQ(tesseract_rosutils::toRcutilsSeverity(spdlog::level::critical), RCUTILS_LOG_SEVERITY_FATAL);
  EXPECT_EQ(tesseract_rosutils::toRcutilsSeverity(spdlog::level::off), RCUTILS_LOG_SEVERITY_UNSET);
}

TEST_F(RosLogRecordHandlerUnit, FormatsStructuredFieldsDeterministically)
{
  tesseract::common::LogRecord record;
  record.message = "changed";
  record.component_name = "environment";
  record.attributes.emplace("object_id", std::string("robot"));
  record.attributes.emplace("endpoint", std::int64_t{ 42 });
  record.attributes.emplace("service_path", std::string("/environment"));
  record.attributes.emplace("member", std::string("modify"));
  record.attributes.emplace("zeta", true);
  record.attributes.emplace("alpha", std::int64_t{ 3 });

  EXPECT_EQ(tesseract_rosutils::formatRosMessage(record),
            "changed component_name=environment alpha=3 endpoint=42 member=modify object_id=robot "
            "service_path=/environment zeta=true");
}

TEST_F(RosLogRecordHandlerUnit, ForwardsLoggerSeverityAndSourceLocation)
{
  tesseract_rosutils::RosLogRecordHandler handler;
  ASSERT_TRUE(handler.start());
  EXPECT_FALSE(handler.start());

  auto logger = tesseract::common::getLogger("test.ros");
  logger->set_level(spdlog::level::trace);
  TESSERACT_LOG_WARN_NAMED("test.ros", "message {}", 7);

  const CapturedLog& captured_log = getCapturedLog();
  EXPECT_EQ(captured_log.severity, RCUTILS_LOG_SEVERITY_WARN);
  EXPECT_EQ(captured_log.name, "test.ros");
  EXPECT_NE(captured_log.file.find("log_record_handler_unit.cpp"), std::string::npos);
  EXPECT_GT(captured_log.line, 0);
  EXPECT_EQ(captured_log.message, "message 7");

  handler.stop();
  EXPECT_FALSE(handler.isStarted());
}