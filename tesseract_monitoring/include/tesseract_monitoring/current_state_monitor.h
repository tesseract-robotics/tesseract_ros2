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

#ifndef TESSERACT_MONITORING_CURRENT_STATE_H
#define TESSERACT_MONITORING_CURRENT_STATE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <functional>
#include <unordered_map>
#include <map>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_monitoring/constants.h>

#include <tesseract_scene_graph/scene_state.h>

#include <tesseract_environment/fwd.h>

namespace tesseract_monitoring
{
using JointStateUpdateCallback = std::function<void(const sensor_msgs::msg::JointState::ConstSharedPtr joint_state)>;

/** @class CurrentStateMonitor
    @brief Monitors the joint_states topic and tf to maintain the current state of the robot. */
class CurrentStateMonitor
{
public:
  using Ptr = std::shared_ptr<CurrentStateMonitor>;
  using ConstPtr = std::shared_ptr<const CurrentStateMonitor>;
  using UPtr = std::unique_ptr<CurrentStateMonitor>;
  using ConstUPtr = std::unique_ptr<const CurrentStateMonitor>;

  /**
   * @brief Constructor.
   * @param robot_model The current kinematic model to build on
   * @param tf A pointer to the tf transformer to use
   */
  CurrentStateMonitor(const std::shared_ptr<const tesseract_environment::Environment>& env);

  /** @brief Constructor.
   *  @param robot_model The current kinematic model to build on
   *  @param tf A pointer to the tf transformer to use
   *  @param node A rclcpp::Node to access ROS info
   */
  CurrentStateMonitor(const std::shared_ptr<const tesseract_environment::Environment>& env,
                      rclcpp::Node::SharedPtr node);

  ~CurrentStateMonitor();
  CurrentStateMonitor(const CurrentStateMonitor&) = delete;
  CurrentStateMonitor& operator=(const CurrentStateMonitor&) = delete;
  CurrentStateMonitor(CurrentStateMonitor&&) = delete;
  CurrentStateMonitor& operator=(CurrentStateMonitor&&) = delete;

  /** @brief Start monitoring joint states on a particular topic
   *  @param joint_states_topic The topic name for joint states (defaults to "joint_states")
   */
  void startStateMonitor(const std::string& joint_states_topic = tesseract_monitoring::DEFAULT_JOINT_STATES_TOPIC);

  /** @brief Stop monitoring the "joint_states" topic
   */
  void stopStateMonitor();

  /** @brief Check if the state monitor is started */
  bool isActive() const;

  /** @brief Get the RobotModel for which we are monitoring state */
  const tesseract_environment::Environment& getEnvironment() const;

  /** @brief Get the name of the topic being monitored. Returns an empty string if the monitor is inactive. */
  std::string getMonitoredTopic() const;

  /** @brief Query whether we have joint state information for all DOFs in the kinematic model
   *  @return False if we have no joint state information for one or more of the joints
   */
  bool haveCompleteState() const;

  /** @brief Query whether we have joint state information for all DOFs in the kinematic model
   *  @return False if we have no joint state information for one of the joints or if our state
   *  information is more than \e age old
   */
  bool haveCompleteState(const rclcpp::Duration& age) const;

  /** @brief Query whether we have joint state information for all DOFs in the kinematic model
   *  @param missing_joints Returns the list of joints that are missing
   *  @return False if we have no joint state information for one or more of the joints
   */
  bool haveCompleteState(std::vector<std::string>& missing_joints) const;

  /** @brief Query whether we have joint state information for all DOFs in the kinematic model
   *  @param age The max allowed age of the joint state information
   *  @param missing_states Returns the list of joints that are missing
   *  @return False if we have no joint state information for one of the joints or if our state
   *  information is more than \e age old*/
  bool haveCompleteState(const rclcpp::Duration& age, std::vector<std::string>& missing_states) const;

  /** @brief Get the current state
   *  @return Returns the current state */
  tesseract_scene_graph::SceneState getCurrentState() const;

  /** @brief Get the time stamp for the current state */
  rclcpp::Time getCurrentStateTime() const;

  /** @brief Get the current state and its time stamp
   *  @return Returns a pair of the current state and its time stamp */
  std::pair<tesseract_scene_graph::SceneState, rclcpp::Time> getCurrentStateAndTime() const;

  /** @brief Get the current state values as a map from joint names to joint state values
   *  @return Returns the map from joint names to joint state values*/
  std::unordered_map<std::string, double> getCurrentStateValues() const;

  /** @brief Wait for at most \e wait_time seconds (default 1s) for a robot state more recent than t
   *  @return true on success, false if up-to-date robot state wasn't received within \e wait_time
   */
  bool waitForCurrentState(const rclcpp::Time& t, double wait_time = 1.0) const;

  /** @brief Wait for at most \e wait_time seconds until the complete robot state is known.
      @return true if the full state is known */
  bool waitForCompleteState(double wait_time) const;

  /** @brief Wait for at most \e wait_time seconds until the joint values from the group \e group are known. Return true
   * if values for all joints in \e group are known */
  bool waitForCompleteState(const std::string& manip, double wait_time) const;

  /** @brief Get the time point when the monitor was started */
  rclcpp::Time getMonitorStartTime() const { return monitor_start_time_; }
  /** @brief Add a function that will be called whenever the joint state is updated*/
  void addUpdateCallback(const JointStateUpdateCallback& fn);

  /** @brief Clear the functions to be called when an update to the joint state is received */
  void clearUpdateCallbacks();

  /** @brief When a joint value is received to be out of bounds, it is changed slightly to fit within bounds,
   *  if the difference is less than a specified value (labeled the "allowed bounds error").
   *  This value can be set using this function.
   *  @param error The specified value for the "allowed bounds error". The default is machine precision. */
  void setBoundsError(double error) { error_ = (error > 0) ? error : -error; }
  /** @brief When a joint value is received to be out of bounds, it is changed slightly to fit within bounds,
   *  if the difference is less than a specified value (labeled the "allowed bounds error").
   *  @return The stored value for the "allowed bounds error"
   */
  double getBoundsError() const { return error_; }
  /** @brief Allow the joint_state arrays velocity and effort to be copied into the robot state
   *  this is useful in some but not all applications
   */
  void enableCopyDynamics(bool enabled) { copy_dynamics_ = enabled; }

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::ConstSharedPtr joint_state);  // NOLINT
  bool isPassiveOrMimicDOF(const std::string& dof) const;

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<const tesseract_environment::Environment> env_;
  tesseract_scene_graph::SceneState env_state_;
  int last_environment_revision_;
  std::map<std::string, rclcpp::Time> joint_time_;
  bool state_monitor_started_;
  bool copy_dynamics_;  // Copy velocity and effort from joint_state
  rclcpp::Time monitor_start_time_;
  double error_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
  rclcpp::Time current_state_time_;

  mutable std::mutex state_update_lock_;
  mutable std::condition_variable state_update_condition_;
  std::vector<JointStateUpdateCallback> update_callbacks_;
};

}  // namespace tesseract_monitoring

#endif
