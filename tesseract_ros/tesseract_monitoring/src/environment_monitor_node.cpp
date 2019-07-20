#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <rclcpp/rclcpp.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/environment.h>
#include <tesseract_monitoring/environment_monitor.h>

using namespace tesseract_environment;

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description"; /**< Default ROS parameter for robot description */

int main(int argc, char** argv)
{
//  rclcpp::init(argc, argv);
////  auto node = rclcpp::Node::make_shared("tesseract_environment_monitor");

//////  ros::init(argc, argv, "tesseract_environment_monitor");
//////  ros::NodeHandle pnh("~");
//  std::string robot_description;
//  std::string descrete_plugin;
//  std::string continuous_plugin;
//  std::string joint_state_topic;
//  std::string monitored_environment_topic;

//  auto monitor = tesseract_monitoring::EnvironmentMonitor::make_shared(robot_description, "", descrete_plugin, continuous_plugin);

////  node->declare_parameter("robot_description");
////  node->declare_parameter("descrete_plugin");  // BUG: typo
////  node->declare_parameter("continuous_plugin");
////  node->declare_parameter("joint_state_topic");
////  node->declare_parameter("monitored_environment_topic");

//  node->get_parameter_or<std::string>("robot_description", robot_description, "");
//  node->get_parameter_or<std::string>("descrete_plugin", descrete_plugin, "");  // BUG: typo
//  node->get_parameter_or<std::string>("continuous_plugin", continuous_plugin, "");
//  node->get_parameter_or<std::string>("joint_state_topic", joint_state_topic, "");
//  node->get_parameter_or<std::string>("monitored_environment_topic", monitored_environment_topic, "");

////  tesseract_monitoring::EnvironmentMonitor monitor(node, robot_description, "", descrete_plugin, continuous_plugin);

//  if (monitored_environment_topic.empty())
//    monitor.startPublishingEnvironment(tesseract_monitoring::EnvironmentMonitor::UPDATE_ENVIRONMENT);
//  else
//    monitor.startPublishingEnvironment(tesseract_monitoring::EnvironmentMonitor::UPDATE_ENVIRONMENT,
//                                       monitored_environment_topic);

//  if (joint_state_topic.empty())
//    monitor.startStateMonitor();
//  else
//    monitor.startStateMonitor(joint_state_topic);

//////  ros::AsyncSpinner spinner(4);
//////  spinner.start();

//  rclcpp::spin(monitor);
//  rclcpp::shutdown();
  return 0;
}
