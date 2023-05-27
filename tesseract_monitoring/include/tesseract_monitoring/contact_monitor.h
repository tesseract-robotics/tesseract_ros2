/**
 * @file contact_monitor.h
 * @brief definition of the contact_monitor library.  It publishes
 * info about which links are (almost) in collision, and how far from/in
 * collision they are.
 *
 * @author David Merz, Jr.
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef TESSERACT_MONITORING_CONTACT_MONITOR_H
#define TESSERACT_MONITORING_CONTACT_MONITOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tesseract_msgs/srv/compute_contact_result_vector.hpp>
#include <tesseract_msgs/srv/modify_environment.hpp>
#include <tesseract_msgs/msg/environment_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <mutex>
#include <condition_variable>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_monitoring/constants.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_environment/environment.h>
#include <tesseract_environment/environment_monitor.h>

namespace tesseract_monitoring
{
class ContactMonitor
{
public:
  ContactMonitor(std::string monitor_namespace,
                 tesseract_environment::Environment::UPtr env,
                 rclcpp::Node::SharedPtr node,
                 std::vector<std::string> monitored_link_names,
                 std::vector<std::string> disabled_link_names,
                 tesseract_collision::ContactTestType type,
                 double contact_distance = 0.1,
                 const std::string& joint_state_topic = DEFAULT_JOINT_STATES_TOPIC);
  ~ContactMonitor();
  /**
   * @brief Custom copy constructor, copy assignment, move constructor, and move
   * assignment.  Because the condition variable current_joint_states_evt_
   * requires a 'cleanup' function call, a custom destructor is required.
   * When a custom destructor is required, it is best to explicitly create
   * these functions.  Here, we do so by assigning them to default values.
   */
  ContactMonitor(const ContactMonitor&) = delete;
  ContactMonitor& operator=(const ContactMonitor&) = delete;
  ContactMonitor(ContactMonitor&&) = delete;
  ContactMonitor& operator=(ContactMonitor&&) = delete;

  /**
   * @brief Start publishing the contact monitors environment
   * @param topic The topic to publish the contact monitor environment on
   */
  void startPublishingEnvironment();

  /**
   * @brief Start monitoring an environment for applying changes to this environment
   * @param topic The topic to monitor for environment changes
   */
  void startMonitoringEnvironment(const std::string& monitored_namespace);

  /**
   * @brief Start publishing the contact markers
   * @param topic The topic topic to publish the contact results markers on
   */
  void startPublishingMarkers();

  /**
   * @brief Compute collision results and publish results.
   *
   * This also publishes environment and contact markers if correct flags are enabled for visualization and debugging.
   */
  void computeCollisionReportThread();

  void callbackJointState(std::shared_ptr<sensor_msgs::msg::JointState> msg);

  void callbackModifyTesseractEnv(tesseract_msgs::srv::ModifyEnvironment::Request::SharedPtr request,
                                  tesseract_msgs::srv::ModifyEnvironment::Response::SharedPtr response);

  void
  callbackComputeContactResultVector(tesseract_msgs::srv::ComputeContactResultVector::Request::SharedPtr request,
                                     tesseract_msgs::srv::ComputeContactResultVector::Response::SharedPtr response);

  void callbackTesseractEnvDiff(const tesseract_msgs::msg::EnvironmentState::SharedPtr state);

private:
  std::string monitor_namespace_;
  std::string monitored_namespace_;
  int env_revision_{ 0 };
  tesseract_environment::EnvironmentMonitor::UPtr monitor_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr internal_node_;
  std::vector<std::string> monitored_link_names_;
  std::vector<std::string> disabled_link_names_;
  tesseract_collision::ContactTestType type_;
  double contact_distance_;
  tesseract_collision::DiscreteContactManager::UPtr manager_;
  bool publish_contact_markers_{ false };
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Publisher<tesseract_msgs::msg::ContactResultVector>::SharedPtr contact_results_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr contact_marker_pub_;
  rclcpp::Service<tesseract_msgs::srv::ComputeContactResultVector>::SharedPtr compute_contact_results_;
  std::mutex modify_mutex_;
  std::shared_ptr<sensor_msgs::msg::JointState> current_joint_states_;
  std::condition_variable current_joint_states_evt_;
};

}  // namespace tesseract_monitoring

#endif  // TESSERACT_MONITORING_
