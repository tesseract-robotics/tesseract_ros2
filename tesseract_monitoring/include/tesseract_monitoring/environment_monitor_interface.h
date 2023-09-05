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
#include <tesseract_msgs/msg/environment_command.hpp>
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

  ROSEnvironmentMonitorInterface(rclcpp::Node::SharedPtr node, const std::string env_name);
  virtual ~ROSEnvironmentMonitorInterface() override = default;
  ROSEnvironmentMonitorInterface(const ROSEnvironmentMonitorInterface&) = default;
  ROSEnvironmentMonitorInterface& operator=(const ROSEnvironmentMonitorInterface&) = default;
  ROSEnvironmentMonitorInterface(ROSEnvironmentMonitorInterface&&) = default;
  ROSEnvironmentMonitorInterface& operator=(ROSEnvironmentMonitorInterface&&) = default;

  bool wait(std::chrono::duration<double> duration = std::chrono::seconds(0)) const override final;

  bool waitForNamespace(const std::string& monitor_namespace,
                        std::chrono::duration<double> duration = std::chrono::seconds(0)) const override final;

  void addNamespace(std::string monitor_namespace) override final;

  void removeNamespace(const std::string& monitor_namespace) override final;

  std::vector<std::string> applyCommand(const tesseract_environment::Command& command) const override final;
  std::vector<std::string> applyCommands(const tesseract_environment::Commands& commands) const override final;
  std::vector<std::string>
  applyCommands(const std::vector<tesseract_environment::Command>& commands) const override final;

  bool applyCommand(const std::string& monitor_namespace,
                    const tesseract_environment::Command& command) const override final;
  bool applyCommands(const std::string& monitor_namespace,
                     const tesseract_environment::Commands& commands) const override final;
  bool applyCommands(const std::string& monitor_namespace,
                     const std::vector<tesseract_environment::Command>& commands) const override final;

  tesseract_scene_graph::SceneState getEnvironmentState(const std::string& monitor_namespace) const override final;

  bool setEnvironmentState(const std::string& monitor_namespace,
                           const std::unordered_map<std::string, double>& joints) const override final;
  bool setEnvironmentState(const std::string& monitor_namespace,
                           const std::vector<std::string>& joint_names,
                           const std::vector<double>& joint_values) const override final;
  bool setEnvironmentState(const std::string& monitor_namespace,
                           const std::vector<std::string>& joint_names,
                           const Eigen::Ref<const Eigen::VectorXd>& joint_values) const override final;

  std::vector<std::string>
  setEnvironmentState(const std::unordered_map<std::string, double>& joints) const override final;
  std::vector<std::string> setEnvironmentState(const std::vector<std::string>& joint_names,
                                               const std::vector<double>& joint_values) const override final;
  std::vector<std::string>
  setEnvironmentState(const std::vector<std::string>& joint_names,
                      const Eigen::Ref<const Eigen::VectorXd>& joint_values) const override final;

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
