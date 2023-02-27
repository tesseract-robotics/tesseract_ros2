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
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <tesseract_msgs/srv/get_environment_information.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_environment/commands.h>
#include <tesseract_environment/environment.h>
#include <tesseract_environment/environment_monitor_interface.h>

namespace tesseract_monitoring
{
class ROSEnvironmentMonitorInterface : public tesseract_environment::EnvironmentMonitorInterface
{
public:
  using Ptr = std::shared_ptr<ROSEnvironmentMonitorInterface>;
  using ConstPtr = std::shared_ptr<const ROSEnvironmentMonitorInterface>;
  using UPtr = std::unique_ptr<ROSEnvironmentMonitorInterface>;
  using ConstUPtr = std::unique_ptr<const ROSEnvironmentMonitorInterface>;

  ROSEnvironmentMonitorInterface(rclcpp::Node::SharedPtr node, const std::string& env_name);
  virtual ~ROSEnvironmentMonitorInterface() override = default;
  ROSEnvironmentMonitorInterface(const ROSEnvironmentMonitorInterface&) = default;
  ROSEnvironmentMonitorInterface& operator=(const ROSEnvironmentMonitorInterface&) = default;
  ROSEnvironmentMonitorInterface(ROSEnvironmentMonitorInterface&&) = default;
  ROSEnvironmentMonitorInterface& operator=(ROSEnvironmentMonitorInterface&&) = default;

  /**
   * @brief This will wait for all namespaces to begin publishing
   * @param timeout The duration to wait before returning, if zero it waits indefinitely
   * @return True if namespace is available, otherwise false
   */
  bool wait(rclcpp::Duration timeout = rclcpp::Duration::from_seconds(0)) const;

  /**
   * @brief This will wait for a given namespace to begin publishing
   * @param monitor_namespace The namespace to wait for
   * @param timeout The duration to wait before returning, if zero it waits indefinitely
   * @return True if namespace is available, otherwise false
   */
  bool waitForNamespace(const std::string& monitor_namespace,
                        rclcpp::Duration timeout = rclcpp::Duration::from_seconds(0)) const;

  /**
   * @brief Add monitor namespace to interface
   * @param monitor_namespace
   */
  void addNamespace(std::string monitor_namespace) override final;

  /**
   * @brief Remove monitor namespace from interface
   * @param monitor_namespace
   */
  void removeNamespace(const std::string& monitor_namespace) override final;

  /**
   * @brief Apply provided command to all monitor namespaces
   * @param command The command to apply
   * @return A vector of failed namespace, if empty all namespace were updated successfully.
   */
  std::vector<std::string> applyCommand(const tesseract_environment::Command& command) const override final;
  std::vector<std::string> applyCommands(const tesseract_environment::Commands& commands) const override final;
  std::vector<std::string>
  applyCommands(const std::vector<tesseract_environment::Command>& commands) const override final;

  /**
   * @brief Apply provided command to only the provided namespace. The namespace does not have to be one that is
   * currently stored in this class.
   * @param command The command to apply
   * @return True if successful, otherwise false
   */
  bool applyCommand(const std::string& monitor_namespace,
                    const tesseract_environment::Command& command) const override final;
  bool applyCommands(const std::string& monitor_namespace,
                     const tesseract_environment::Commands& commands) const override final;
  bool applyCommands(const std::string& monitor_namespace,
                     const std::vector<tesseract_environment::Command>& commands) const override final;

  /**
   * @brief Pull current environment state from the environment in the provided namespace
   * @param monitor_namespace The namespace to extract the environment from.
   * @return Environment Shared Pointer, if nullptr it failed
   */
  tesseract_scene_graph::SceneState getEnvironmentState(const std::string& monitor_namespace) const override final;

  /**
   * @brief Set environments state in the provided namespace
   * @param monitor_namespace The monitored namespace to set the state
   * @return True if successful, otherwise false
   */
  bool setEnvironmentState(const std::string& monitor_namespace,
                           const std::unordered_map<std::string, double>& joints) const override final;
  bool setEnvironmentState(const std::string& monitor_namespace,
                           const std::vector<std::string>& joint_names,
                           const std::vector<double>& joint_values) const override final;
  bool setEnvironmentState(const std::string& monitor_namespace,
                           const std::vector<std::string>& joint_names,
                           const Eigen::Ref<const Eigen::VectorXd>& joint_values) const override final;

  /**
   * @brief Set environment state for all monitor namespaces
   * @return A vector of failed namespace, if empty all namespace were updated successfully.
   */
  std::vector<std::string>
  setEnvironmentState(const std::unordered_map<std::string, double>& joints) const override final;
  std::vector<std::string> setEnvironmentState(const std::vector<std::string>& joint_names,
                                               const std::vector<double>& joint_values) const override final;
  std::vector<std::string>
  setEnvironmentState(const std::vector<std::string>& joint_names,
                      const Eigen::Ref<const Eigen::VectorXd>& joint_values) const override final;

  /**
   * @brief Pull information from the environment in the provided namespace and create a Environment Object
   * @param monitor_namespace The namespace to extract the environment from.
   * @return Environment Shared Pointer, if nullptr it failed
   */
  tesseract_environment::Environment::UPtr getEnvironment(const std::string& monitor_namespace) const override final;

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
