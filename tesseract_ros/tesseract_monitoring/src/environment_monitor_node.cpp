#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <rclcpp/rclcpp.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_monitoring/environment_monitor.h>

using namespace tesseract_environment;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::sleep_for(std::chrono::seconds(3));
  auto monitor = std::make_shared<tesseract_monitoring::EnvironmentMonitor>("env");
  monitor->postInitialize();

//  rclcpp::executors::MultiThreadedExecutor multi_executor;
//  multi_executor.add_node(monitor);
//  multi_executor.spin();

  rclcpp::spin(monitor);
  rclcpp::shutdown();
  monitor.reset();
  return 0;
}
