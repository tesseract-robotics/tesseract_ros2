/**
 * @file tesseract_monitor_interface.cpp
 * @brief This is a utility class for applying changes to multiple tesseract monitors
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <rclcpp/rclcpp.hpp>
#include <tesseract_msgs/srv/get_environment_information.hpp>
#include <tesseract_msgs/srv/modify_environment.hpp>
#include <tesseract_msgs/msg/environment_command.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_monitoring/environment_monitor_interface.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_monitoring/constants.h>

namespace tesseract_monitoring
{
using namespace std::chrono_literals;

/**
 * Call and wait for service by running a new executor which only operates on one callback group
 */
template <class SrvType>
typename SrvType::Response::SharedPtr call_service(const std::string& name,
                                                   typename SrvType::Request::SharedPtr request,
                                                   rclcpp::Node& node,
                                                   rclcpp::CallbackGroup::SharedPtr cbg,
                                                   std::chrono::duration<double> timeout)
{
  auto client = node.create_client<SrvType>(name, ::rmw_qos_profile_services_default, cbg);
  if (!client->service_is_ready())
  {
    RCLCPP_ERROR_STREAM(node.get_logger(), "Service '" << name << "' not available!");
    return nullptr;
  }

  rclcpp::executors::SingleThreadedExecutor exec;
#if __has_include(<rclcpp/version.h>)  // ROS 2 Humble
  exec.add_callback_group(cbg, node.get_node_base_interface());
#else  // ROS 2 Foxy
  exec.add_node(node.get_node_base_interface());
#endif

  auto future = client->async_send_request(request);
  auto retcode = exec.spin_until_future_complete(future, timeout);
  if (retcode != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(node.get_logger(), "No response received for service '" << name << "'");
    return nullptr;
  }

  return future.get();
}

ROSEnvironmentMonitorInterface::ROSEnvironmentMonitorInterface(rclcpp::Node::SharedPtr node, const std::string env_name)
  : EnvironmentMonitorInterface(std::move(env_name)), node_
{
  node
}
#if __has_include(<rclcpp/version.h>)  // ROS 2 Humble
, callback_group_ { node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false) }
#else  // ROS 2 Foxy
, callback_group_ { node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive) }
#endif
, logger_{ node_->get_logger().get_child(env_name + "_env_monitor") }, env_name_{ env_name } {}

bool ROSEnvironmentMonitorInterface::wait(std::chrono::duration<double> duration) const
{
  if (ns_.empty())
  {
    RCLCPP_ERROR(logger_, "TesseractMonitorInterface namespaces are empty and cannot wait!");
    return false;
  }

  const auto start_time = rclcpp::Clock().now();
  rclcpp::Duration wall_timeout(duration);
  if (std::chrono::duration_cast<std::chrono::seconds>(duration).count() == 0)
    wall_timeout = rclcpp::Duration::from_seconds(-1);

  for (const auto& ns : ns_)
  {
    bool results = waitForNamespace(ns, duration);
    if (!results)
      return false;

    if (wall_timeout >= rclcpp::Duration::from_seconds(0))
    {
      const auto current_time = rclcpp::Clock().now();

      if ((current_time - start_time) >= wall_timeout)
        return false;
    }
  }
  return true;
}

bool ROSEnvironmentMonitorInterface::waitForNamespace(const std::string& monitor_namespace,
                                                      std::chrono::duration<double> duration) const
{
  std::string srv_name = R"(/)" + monitor_namespace + DEFAULT_GET_ENVIRONMENT_INFORMATION_SERVICE;
  const auto start_time = rclcpp::Clock().now();
  rclcpp::Duration wall_timeout(duration);
  if (std::chrono::duration_cast<std::chrono::seconds>(duration).count() == 0)
    wall_timeout = rclcpp::Duration::from_seconds(-1);

  auto req = std::make_shared<tesseract_msgs::srv::GetEnvironmentInformation::Request>();
  req->flags = tesseract_msgs::srv::GetEnvironmentInformation::Request::COMMAND_HISTORY;
  while (rclcpp::ok())
  {
    try
    {
      auto res =
          call_service<tesseract_msgs::srv::GetEnvironmentInformation>(srv_name, req, *node_, callback_group_, 5s);
      if (res && res->success)
        return true;
    }
    catch (std::runtime_error& ex)
    {
      // RCLCPP_ERROR_STREAM(logger_, ex.what());
    }
    if (wall_timeout >= rclcpp::Duration::from_seconds(0))
    {
      const auto current_time = rclcpp::Clock().now();

      if ((current_time - start_time) >= wall_timeout)
        return false;
    }

    rclcpp::sleep_for(20ms);
  }

  return false;
}

void ROSEnvironmentMonitorInterface::addNamespace(std::string monitor_namespace)
{
  if (std::find(ns_.begin(), ns_.end(), monitor_namespace) == ns_.end())
    ns_.push_back(monitor_namespace);
}

void ROSEnvironmentMonitorInterface::removeNamespace(const std::string& monitor_namespace)
{
  auto it = std::remove_if(
      ns_.begin(), ns_.end(), [monitor_namespace](const std::string& ns) { return (ns == monitor_namespace); });
  ns_.erase(it, ns_.end());
}

std::vector<std::string>
ROSEnvironmentMonitorInterface::applyCommand(const tesseract_environment::Command& command) const
{
  std::vector<std::string> failed_namespace;
  failed_namespace.reserve(ns_.size());
  for (const auto& ns : ns_)
    if (!applyCommand(ns, command))
      failed_namespace.push_back(ns);

  return failed_namespace;
}

std::vector<std::string>
ROSEnvironmentMonitorInterface::applyCommands(const tesseract_environment::Commands& commands) const
{
  std::vector<std::string> failed_namespace;
  failed_namespace.reserve(ns_.size());
  for (const auto& ns : ns_)
    if (!applyCommands(ns, commands))
      failed_namespace.push_back(ns);

  return failed_namespace;
}

std::vector<std::string>
ROSEnvironmentMonitorInterface::applyCommands(const std::vector<tesseract_environment::Command>& commands) const
{
  std::vector<std::string> failed_namespace;
  failed_namespace.reserve(ns_.size());
  for (const auto& ns : ns_)
    if (!applyCommands(ns, commands))
      failed_namespace.push_back(ns);

  return failed_namespace;
}

bool ROSEnvironmentMonitorInterface::applyCommand(const std::string& monitor_namespace,
                                                  const tesseract_environment::Command& command) const
{
  tesseract_msgs::msg::EnvironmentCommand command_msg;
  if (tesseract_rosutils::toMsg(command_msg, command))
  {
    return sendCommands(monitor_namespace, { command_msg });
  }
  else
  {
    RCLCPP_ERROR_STREAM(node_->get_logger().get_child("environment_monitor_interface").get_child(monitor_namespace),
                        "Failed to convert latest changes to message and update monitored environment!");
    return false;
  }
}

bool ROSEnvironmentMonitorInterface::applyCommands(const std::string& monitor_namespace,
                                                   const tesseract_environment::Commands& commands) const
{
  std::vector<tesseract_msgs::msg::EnvironmentCommand> commands_msg;
  if (tesseract_rosutils::toMsg(commands_msg, commands, 0))
  {
    return sendCommands(monitor_namespace, commands_msg);
  }
  else
  {
    RCLCPP_ERROR_STREAM(node_->get_logger().get_child("environment_monitor_interface").get_child(monitor_namespace),
                        "Failed to convert latest changes to message and update monitored environment!");
    return false;
  }
}

bool ROSEnvironmentMonitorInterface::applyCommands(const std::string& monitor_namespace,
                                                   const std::vector<tesseract_environment::Command>& commands) const
{
  std::vector<tesseract_msgs::msg::EnvironmentCommand> commands_msg;
  commands_msg.reserve(commands.size());
  for (const auto& cmd : commands)
  {
    tesseract_msgs::msg::EnvironmentCommand cmd_msg;
    if (tesseract_rosutils::toMsg(cmd_msg, cmd))
    {
      commands_msg.push_back(cmd_msg);
    }
    else
    {
      RCLCPP_ERROR_STREAM(node_->get_logger().get_child("environment_monitor_interface").get_child(monitor_namespace),
                          "Failed to convert latest changes to message and update monitored environment!");
      return false;
    }
  }

  return sendCommands(monitor_namespace, commands_msg);
}

bool ROSEnvironmentMonitorInterface::sendCommands(
    const std::string& ns,
    const std::vector<tesseract_msgs::msg::EnvironmentCommand>& commands) const
{
  auto req = std::make_shared<tesseract_msgs::srv::ModifyEnvironment::Request>();
  req->id = env_name_;
  req->append = true;
  req->commands = commands;

  try
  {
    const std::string srv_name = R"(/)" + ns + DEFAULT_MODIFY_ENVIRONMENT_SERVICE;
    auto response = call_service<tesseract_msgs::srv::ModifyEnvironment>(srv_name, req, *node_, callback_group_, 10s);
    if (!response || !response->success)
    {
      RCLCPP_ERROR_STREAM(logger_, "sendCommands (" + ns + "): Failed to update monitored environment!");
      return false;
    }
  }
  catch (std::runtime_error& ex)
  {
    RCLCPP_ERROR_STREAM(logger_, "sendCommands (" + ns + "): Failed - " << ex.what());
    return false;
  }
  return true;
}

tesseract_scene_graph::SceneState
ROSEnvironmentMonitorInterface::getEnvironmentState(const std::string& monitor_namespace) const
{
  auto req = std::make_shared<tesseract_msgs::srv::GetEnvironmentInformation::Request>();
  req->flags = tesseract_msgs::srv::GetEnvironmentInformation::Request::JOINT_STATES |
               tesseract_msgs::srv::GetEnvironmentInformation::Request::LINK_TRANSFORMS |
               tesseract_msgs::srv::GetEnvironmentInformation::Request::JOINT_TRANSFORMS;

  try
  {
    const std::string srv_name = R"(/)" + monitor_namespace + DEFAULT_GET_ENVIRONMENT_INFORMATION_SERVICE;
    auto res = call_service<tesseract_msgs::srv::GetEnvironmentInformation>(srv_name, req, *node_, callback_group_, 3s);
    if (!res || !res->success)
      throw std::runtime_error("getEnvironmentState: Failed to get monitor environment information!");
    tesseract_scene_graph::SceneState env_state;
    tesseract_rosutils::fromMsg(env_state.joints, res->joint_states);
    tesseract_rosutils::fromMsg(env_state.link_transforms, res->link_transforms);
    tesseract_rosutils::fromMsg(env_state.joint_transforms, res->joint_transforms);
    return env_state;
  }
  catch (std::runtime_error& ex)
  {
    throw std::runtime_error(std::string{ "getEnvironmentState: Failed - " } + ex.what());
  }
}

bool ROSEnvironmentMonitorInterface::setEnvironmentState(const std::string& monitor_namespace,
                                                         const std::unordered_map<std::string, double>& joints) const
{
  tesseract_msgs::msg::EnvironmentCommand command;
  command.command = tesseract_msgs::msg::EnvironmentCommand::UPDATE_JOINT_STATE;
  tesseract_rosutils::toMsg(command.joint_state, joints);
  return sendCommands(monitor_namespace, { command });
}

bool ROSEnvironmentMonitorInterface::setEnvironmentState(const std::string& monitor_namespace,
                                                         const std::vector<std::string>& joint_names,
                                                         const std::vector<double>& joint_values) const
{
  std::unordered_map<std::string, double> joints;
  for (std::size_t i = 0; i < joint_names.size(); ++i)
    joints[joint_names[i]] = joint_values[i];

  tesseract_msgs::msg::EnvironmentCommand command;
  command.command = tesseract_msgs::msg::EnvironmentCommand::UPDATE_JOINT_STATE;
  tesseract_rosutils::toMsg(command.joint_state, joints);
  return sendCommands(monitor_namespace, { command });
}

bool ROSEnvironmentMonitorInterface::setEnvironmentState(const std::string& monitor_namespace,
                                                         const std::vector<std::string>& joint_names,
                                                         const Eigen::Ref<const Eigen::VectorXd>& joint_values) const
{
  std::unordered_map<std::string, double> joints;
  for (std::size_t i = 0; i < joint_names.size(); ++i)
    joints[joint_names[i]] = joint_values[static_cast<Eigen::Index>(i)];

  tesseract_msgs::msg::EnvironmentCommand command;
  command.command = tesseract_msgs::msg::EnvironmentCommand::UPDATE_JOINT_STATE;
  tesseract_rosutils::toMsg(command.joint_state, joints);
  return sendCommands(monitor_namespace, { command });
}

std::vector<std::string>
ROSEnvironmentMonitorInterface::setEnvironmentState(const std::unordered_map<std::string, double>& joints) const
{
  std::vector<std::string> failed_namespace;
  failed_namespace.reserve(ns_.size());
  for (const auto& ns : ns_)
    if (!setEnvironmentState(ns, joints))
      failed_namespace.push_back(ns);

  return failed_namespace;
}

std::vector<std::string>
ROSEnvironmentMonitorInterface::setEnvironmentState(const std::vector<std::string>& joint_names,
                                                    const std::vector<double>& joint_values) const
{
  std::vector<std::string> failed_namespace;
  failed_namespace.reserve(ns_.size());
  for (const auto& ns : ns_)
    if (!setEnvironmentState(ns, joint_names, joint_values))
      failed_namespace.push_back(ns);

  return failed_namespace;
}

std::vector<std::string>
ROSEnvironmentMonitorInterface::setEnvironmentState(const std::vector<std::string>& joint_names,
                                                    const Eigen::Ref<const Eigen::VectorXd>& joint_values) const
{
  std::vector<std::string> failed_namespace;
  failed_namespace.reserve(ns_.size());
  for (const auto& ns : ns_)
    if (!setEnvironmentState(ns, joint_names, joint_values))
      failed_namespace.push_back(ns);

  return failed_namespace;
}

tesseract_environment::Environment::UPtr
ROSEnvironmentMonitorInterface::getEnvironment(const std::string& monitor_namespace) const
{
  auto req = std::make_shared<tesseract_msgs::srv::GetEnvironmentInformation::Request>();
  req->flags = tesseract_msgs::srv::GetEnvironmentInformation::Request::COMMAND_HISTORY;

  try
  {
    const std::string srv_name = R"(/)" + monitor_namespace + DEFAULT_GET_ENVIRONMENT_INFORMATION_SERVICE;
    auto res = call_service<tesseract_msgs::srv::GetEnvironmentInformation>(srv_name, req, *node_, callback_group_, 3s);
    if (!res || !res->success)
    {
      RCLCPP_ERROR_STREAM(logger_, "getEnvironment: Failed to get monitor environment information!");
      return nullptr;
    }
    tesseract_environment::Commands commands;
    commands = tesseract_rosutils::fromMsg(res->command_history);
    auto env = std::make_unique<tesseract_environment::Environment>();
    env->init(commands);
    return env;
  }
  catch (std::runtime_error& ex)
  {
    RCLCPP_ERROR_STREAM(logger_, "getEnvironment: Failed to convert command history message!" << ex.what());
    return nullptr;
  }
}

}  // namespace tesseract_monitoring
