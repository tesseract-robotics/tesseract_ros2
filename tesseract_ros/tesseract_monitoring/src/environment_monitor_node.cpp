#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <rclcpp/rclcpp.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_monitoring/environment_monitor.h>

using namespace tesseract_environment;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("env_monitor");
  auto monitor = std::make_shared<tesseract_monitoring::EnvironmentMonitor>("env", node);
  monitor->postInitialize();

  rclcpp::spin(node);
  rclcpp::shutdown();
  monitor.reset();
  return 0;
}
