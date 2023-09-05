/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#ifndef TESSERACT_MONITORING_ENVIRONMENT_H
#define TESSERACT_MONITORING_ENVIRONMENT_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <shared_mutex>
#include <mutex>
#include <thread>
#include <functional>
#include <tesseract_msgs/msg/environment_state.hpp>
#include <tesseract_msgs/srv/modify_environment.hpp>
#include <tesseract_msgs/srv/get_environment_changes.hpp>
#include <tesseract_msgs/srv/get_environment_information.hpp>
#include <tesseract_msgs/srv/save_scene_graph.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/environment.h>
#include <tesseract_environment/environment_monitor.h>
#include <tesseract_monitoring/current_state_monitor.h>

namespace tesseract_monitoring
{
/**
 * @brief TesseractMonitor
 * Subscribes to the topic \e tesseract_environment */
class ROSEnvironmentMonitor : public tesseract_environment::EnvironmentMonitor
{
public:
  using Ptr = std::shared_ptr<ROSEnvironmentMonitor>;
  using ConstPtr = std::shared_ptr<const ROSEnvironmentMonitor>;
  using UPtr = std::unique_ptr<ROSEnvironmentMonitor>;
  using ConstUPtr = std::unique_ptr<const ROSEnvironmentMonitor>;

  /**
   * @brief Constructor
   * @param robot_description The name of the ROS parameter that contains the URDF (in string format)
   * @param monitor_namespace A name identifying this monitor, must be unique
   */
  ROSEnvironmentMonitor(rclcpp::Node::SharedPtr node, std::string robot_description, std::string monitor_namespace);

  /**
   * @brief Constructor
   * @param env The environment
   * @param monitor_namespace A name identifying this monitor, must be unique
   */
  ROSEnvironmentMonitor(rclcpp::Node::SharedPtr node,
                        tesseract_environment::Environment::Ptr env,
                        std::string monitor_namespace);

  ~ROSEnvironmentMonitor() override;
  ROSEnvironmentMonitor(const ROSEnvironmentMonitor&) = delete;
  ROSEnvironmentMonitor& operator=(const ROSEnvironmentMonitor&) = delete;
  ROSEnvironmentMonitor(ROSEnvironmentMonitor&&) = delete;
  ROSEnvironmentMonitor& operator=(ROSEnvironmentMonitor&&) = delete;

  /** @brief Get the stored robot description
   *  @return An instance of the stored robot description*/
  const std::string& getURDFDescription() const;

  bool waitForConnection(std::chrono::duration<double> duration = std::chrono::seconds(0)) const override final;

  void startPublishingEnvironment() override final;

  void stopPublishingEnvironment() override final;

  void setEnvironmentPublishingFrequency(double hz) override final;

  double getEnvironmentPublishingFrequency() const override final;

  void startStateMonitor(const std::string& joint_states_topic = DEFAULT_JOINT_STATES_TOPIC,
                         bool publish_tf = true) override final;

  void stopStateMonitor() override final;

  void setStateUpdateFrequency(double hz = 10) override final;

  double getStateUpdateFrequency() const override final;

  void updateEnvironmentWithCurrentState() override final;

  void startMonitoringEnvironment(const std::string& monitored_namespace,
                                  tesseract_environment::MonitoredEnvironmentMode mode =
                                      tesseract_environment::MonitoredEnvironmentMode::DEFAULT) override final;

  void stopMonitoringEnvironment() override final;

  bool waitForCurrentState(std::chrono::duration<double> duration = std::chrono::seconds(1)) override final;

  void shutdown() override final;

  /** @brief Get the stored instance of the stored current state monitor
   *  @return An instance of the stored current state monitor*/
  const CurrentStateMonitor& getStateMonitor() const;
  CurrentStateMonitor& getStateMonitor();

protected:
  /** @brief Initialize the planning scene monitor
   *  @param scene The scene instance to fill with data (an instance is allocated if the one passed in is not allocated)
   */
  bool initialize();

  rclcpp::Time last_update_time_ = rclcpp::Time(0l, RCL_ROS_TIME);        /// Last time the state was updated
  rclcpp::Time last_robot_motion_time_ = rclcpp::Time(0l, RCL_ROS_TIME);  /// Last time the robot has moved
  bool enforce_next_state_update_;  /// flag to enforce immediate state update in onStateUpdate()

  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr internal_node_;
  rclcpp::executors::MultiThreadedExecutor::SharedPtr internal_node_executor_;
  std::shared_ptr<std::thread> internal_node_spinner_;
  std::string robot_description_;

  // variables for planning scene publishing
  rclcpp::Publisher<tesseract_msgs::msg::EnvironmentState>::SharedPtr environment_publisher_;
  std::unique_ptr<std::thread> publish_environment_;
  double publish_environment_frequency_;

  // variables for monitored environment
  rclcpp::Subscription<tesseract_msgs::msg::EnvironmentState>::SharedPtr monitored_environment_subscriber_;
  rclcpp::Client<tesseract_msgs::srv::GetEnvironmentChanges>::SharedPtr get_monitored_environment_changes_client_;
  rclcpp::Client<tesseract_msgs::srv::ModifyEnvironment>::SharedPtr modify_monitored_environment_client_;
  rclcpp::Client<tesseract_msgs::srv::GetEnvironmentInformation>::SharedPtr
      get_monitored_environment_information_client_;

  // host a service for modifying the environment
  rclcpp::Service<tesseract_msgs::srv::ModifyEnvironment>::SharedPtr modify_environment_service_;

  // host a service for getting the environment changes
  rclcpp::Service<tesseract_msgs::srv::GetEnvironmentChanges>::SharedPtr get_environment_changes_service_;

  // host a service for getting the environment information
  rclcpp::Service<tesseract_msgs::srv::GetEnvironmentInformation>::SharedPtr get_environment_information_service_;

  // host a service for saving the scene graph to a DOT file
  rclcpp::Service<tesseract_msgs::srv::SaveSceneGraph>::SharedPtr save_scene_graph_service_;

  // include a current state monitor
  CurrentStateMonitor::UPtr current_state_monitor_;

  /// lock access to update_callbacks_
  std::recursive_mutex update_lock_;

  /// List of callbacks to trigger when updates are received
  std::vector<std::function<void()> > update_callbacks_;

  rclcpp::CallbackGroup::SharedPtr cb_group_;

private:
  /** @brief Handle state changes (when not monitoring) */
  void sceneStateChangedCallback(const tesseract_environment::Event& event);

  // publish environment update diffs (runs in its own thread)
  void environmentPublishingThread();

  // called by current_state_monitor_ when robot state (as monitored on joint state topic) changes
  void onJointStateUpdate(const sensor_msgs::msg::JointState::ConstSharedPtr joint_state);

  // called by state_update_timer_ when a state update is pending
  void updateJointStateTimerCallback();

  // Callback for a new state msg
  void newEnvironmentStateCallback(const tesseract_msgs::msg::EnvironmentState::ConstSharedPtr env);

  /** @brief Callback for modifying the environment via service request */
  void modifyEnvironmentCallback(tesseract_msgs::srv::ModifyEnvironment::Request::SharedPtr req,
                                 tesseract_msgs::srv::ModifyEnvironment::Response::SharedPtr res);

  /** @brief Callback for get the environment changes via service request */
  void getEnvironmentChangesCallback(tesseract_msgs::srv::GetEnvironmentChanges::Request::SharedPtr req,
                                     tesseract_msgs::srv::GetEnvironmentChanges::Response::SharedPtr res);

  /** @brief Callback for get the environment information via service request */
  void getEnvironmentInformationCallback(tesseract_msgs::srv::GetEnvironmentInformation::Request::SharedPtr req,
                                         tesseract_msgs::srv::GetEnvironmentInformation::Response::SharedPtr res);

  /** @brief Callback to save the scene graph to a DOT via a service request */
  void saveSceneGraphCallback(tesseract_msgs::srv::SaveSceneGraph::Request::SharedPtr req,
                              tesseract_msgs::srv::SaveSceneGraph::Response::SharedPtr res);

  // Called when new service request is called to modify the environment.
  bool applyEnvironmentCommandsMessage(const std::string& id,
                                       int revision,
                                       const std::vector<tesseract_msgs::msg::EnvironmentCommand>& commands);

  // Lock for state_update_pending_ and dt_state_update_
  std::mutex state_pending_mutex_;

  /// True when we need to update the RobotState from current_state_monitor_
  // This field is protected by state_pending_mutex_
  volatile bool state_update_pending_;

  /// the amount of time to wait in between updates to the robot state
  // This field is protected by state_pending_mutex_
  rclcpp::Duration dt_state_update_ = rclcpp::Duration::from_seconds(0);

  /// timer for state updates.
  // Check if last_state_update_ is true and if so call updateSceneWithCurrentState()
  // Not safe to access from callback functions.
  rclcpp::TimerBase::SharedPtr state_update_timer_;

  /// Last time the state was updated from current_state_monitor_
  // Only access this from callback functions (and constructor)
  rclcpp::Time last_robot_state_update_wall_time_ = rclcpp::Time(0l, RCL_SYSTEM_TIME);

  std::atomic<bool> publish_{ false };

  rclcpp::Logger logger_;
};

}  // namespace tesseract_monitoring

#endif
