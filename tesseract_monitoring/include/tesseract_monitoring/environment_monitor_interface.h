/**
 * @file environment_monitor_interface.h
 * @brief This is a utility class for applying changes to multiple tesseract environment monitors
 *
 * @author Levi Armstrong
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
#ifndef TESSERACT_MONITORING_ENVIRONMENT_MONITOR_INTERFACE_H
#define TESSERACT_MONITORING_ENVIRONMENT_MONITOR_INTERFACE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

 #include <tesseract_msgs/msg/environment_command.h>
#include <tesseract_environment/commands.h>
#include <tesseract_environment/environment.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_monitoring/constants.h>

#include <rclcpp/rclcpp.hpp>

namespace tesseract_monitoring
{
class EnvironmentMonitorInterface
{
public:
  using Ptr = std::shared_ptr<EnvironmentMonitorInterface>;
  using ConstPtr = std::shared_ptr<const EnvironmentMonitorInterface>;

  EnvironmentMonitorInterface(rclcpp::Node::SharedPtr node, const std::string& env_name);
  virtual ~EnvironmentMonitorInterface() = default;
  EnvironmentMonitorInterface(const EnvironmentMonitorInterface&) = default;
  EnvironmentMonitorInterface& operator=(const EnvironmentMonitorInterface&) = default;
  EnvironmentMonitorInterface(EnvironmentMonitorInterface&&) = default;
  EnvironmentMonitorInterface& operator=(EnvironmentMonitorInterface&&) = default;

  /**
   * @brief This will wait for all namespaces to begin publishing
   * @param timeout The duration to wait before returning, if zero it waits indefinitely
   * @return True if namespace is available, otherwise false
   */
  bool wait(rclcpp::Duration timeout = rclcpp::Duration::from_nanoseconds(-1)) const;

  /**
   * @brief This will wait for a given namespace to begin publishing
   * @param monitor_namespace The namespace to wait for
   * @param timeout The duration to wait before returning, if zero it waits indefinitely
   * @return True if namespace is available, otherwise false
   */
  bool waitForNamespace(const std::string& monitor_namespace, rclcpp::Duration timeout = rclcpp::Duration::from_nanoseconds(-1)) const;

  /**
   * @brief Add monitor namespace to interface
   * @param monitor_namespace
   */
  void addNamespace(std::string monitor_namespace);

  /**
   * @brief Remove monitor namespace from interface
   * @param monitor_namespace
   */
  void removeNamespace(const std::string& monitor_namespace);

  /**
   * @brief Apply provided command to all monitor namespaces
   * @param command The command to apply
   * @return A vector of failed namespace, if empty all namespace were updated successfully.
   */
  std::vector<std::string> applyCommand(const tesseract_environment::Command& command) const;
  std::vector<std::string> applyCommands(const tesseract_environment::Commands& commands) const;
  std::vector<std::string> applyCommands(const std::vector<tesseract_environment::Command>& commands) const;

  /**
   * @brief Apply provided command to only the provided namespace. The namespace does not have to be one that is
   * currently stored in this class.
   * @param command The command to apply
   * @return True if successful, otherwise false
   */
  bool applyCommand(const std::string& monitor_namespace, const tesseract_environment::Command& command) const;
  bool applyCommands(const std::string& monitor_namespace, const tesseract_environment::Commands& commands) const;
  bool applyCommands(const std::string& monitor_namespace,
                     const std::vector<tesseract_environment::Command>& commands) const;

  /**
   * @brief Pull current environment state from the environment in the provided namespace
   * @param monitor_namespace The namespace to extract the environment from.
   * @return Environment Shared Pointer, if nullptr it failed
   */
  tesseract_scene_graph::SceneState getEnvironmentState(const std::string& monitor_namespace) const;

  /**
   * @brief Set environments state in the provided namespace
   * @param monitor_namespace The monitored namespace to set the state
   * @return True if successful, otherwise false
   */
  bool setEnvironmentState(const std::string& monitor_namespace,
                           const std::unordered_map<std::string, double>& joints) const;
  bool setEnvironmentState(const std::string& monitor_namespace,
                           const std::vector<std::string>& joint_names,
                           const std::vector<double>& joint_values);
  bool setEnvironmentState(const std::string& monitor_namespace,
                           const std::vector<std::string>& joint_names,
                           const Eigen::Ref<const Eigen::VectorXd>& joint_values);

  /**
   * @brief Set environment state for all monitor namespaces
   * @return A vector of failed namespace, if empty all namespace were updated successfully.
   */
  std::vector<std::string> setEnvironmentState(const std::unordered_map<std::string, double>& joints);
  std::vector<std::string> setEnvironmentState(const std::vector<std::string>& joint_names,
                                               const std::vector<double>& joint_values);
  std::vector<std::string> setEnvironmentState(const std::vector<std::string>& joint_names,
                                               const Eigen::Ref<const Eigen::VectorXd>& joint_values);

  /**
   * @brief Pull information from the environment in the provided namespace and create a Environment Object
   * @param monitor_namespace The namespace to extract the environment from.
   * @return Environment Shared Pointer, if nullptr it failed
   */
  tesseract_environment::Environment::Ptr getEnvironment(const std::string& monitor_namespace);

protected:
  rclcpp::Node::SharedPtr node_;
  // Own a special callback that will be used to receive service replies outside of the main node execution
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Logger logger_;

  std::vector<std::string> ns_;
  std::string env_name_;

  bool sendCommands(const std::string& ns, const std::vector<tesseract_msgs::msg::EnvironmentCommand>& commands) const;
};
}  // namespace tesseract_monitoring
#endif  // TESSERACT_MONITORING_ENVIRONMENT_MONITOR_INTERFACE_H
