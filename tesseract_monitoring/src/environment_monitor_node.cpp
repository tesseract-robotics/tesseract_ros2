#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <rclcpp/rclcpp.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/environment.h>
#include <tesseract_monitoring/environment_monitor.h>

using namespace tesseract_environment;

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description"; /**< Default ROS parameter for robot description */

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("tesseract_environment_monitor");

  std::string monitor_namespace = node->declare_parameter("monitor_namespace", "");

  if (monitor_namespace == "")
  {
    RCLCPP_ERROR(node->get_logger(), "Missing required parameter monitor_namespace!");
    return 1;
  }

  std::string monitored_namespace = node->declare_parameter("monitored_namespace", "");
  std::string robot_description = node->declare_parameter("robot_description", ROBOT_DESCRIPTION_PARAM);
  std::string joint_state_topic = node->declare_parameter("joint_state_topic", "");
  bool publish_environment = node->declare_parameter("publish_environment", false);

  tesseract_monitoring::ROSEnvironmentMonitor monitor(node, robot_description, monitor_namespace);

  if (publish_environment)
    monitor.startPublishingEnvironment();

  if (!monitored_namespace.empty())
    monitor.startMonitoringEnvironment(monitored_namespace);

  if (joint_state_topic.empty())
    monitor.startStateMonitor();
  else
    monitor.startStateMonitor(joint_state_topic);

  rclcpp::spin(node);

  return 0;
}
