#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <rclcpp/rclcpp.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/environment.h>
#include <tesseract_monitoring/environment_monitor.h>

using namespace tesseract_environment;
using namespace tesseract_monitoring;

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description"; /**< Default ROS parameter for robot description */
const std::string ROBOT_DESCRIPTION_SEMANTIC_PARAM = "robot_description_semantic";

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
  std::string robot_description = node->declare_parameter(ROBOT_DESCRIPTION_PARAM, "");
  if (robot_description == "")
  {
    RCLCPP_ERROR(node->get_logger(), "Missing required parameter %s!", ROBOT_DESCRIPTION_PARAM.c_str());
    return 1;
  }
  std::string robot_description_semantic = node->declare_parameter(ROBOT_DESCRIPTION_SEMANTIC_PARAM, "");
  if (robot_description_semantic == "")
  {
    RCLCPP_ERROR(node->get_logger(), "Missing required parameter %s!", ROBOT_DESCRIPTION_SEMANTIC_PARAM.c_str());
    return 1;
  }
  std::string joint_state_topic = node->declare_parameter("joint_state_topic", "");
  bool publish_environment = node->declare_parameter("publish_environment", false);

  tesseract_monitoring::ROSEnvironmentMonitor monitor(node, ROBOT_DESCRIPTION_PARAM, monitor_namespace);

  if (publish_environment)
    monitor.startPublishingEnvironment();

  if (!monitored_namespace.empty())
    monitor.startMonitoringEnvironment(monitored_namespace);

  bool publish_tf = monitored_namespace.empty();
  if (joint_state_topic.empty())
    monitor.startStateMonitor(DEFAULT_JOINT_STATES_TOPIC, publish_tf);
  else
    monitor.startStateMonitor(joint_state_topic, publish_tf);

  RCLCPP_INFO(node->get_logger(), "Environment Monitor Running!");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
