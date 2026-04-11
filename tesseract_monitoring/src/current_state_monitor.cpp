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

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <limits>
#include <geometry_msgs/msg/transform_stamped.hpp>
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_monitoring/current_state_monitor.h>

#include <tesseract/common/types.h>
#include <tesseract/kinematics/joint_group.h>
#include <tesseract/scene_graph/joint.h>
#include <tesseract/environment/environment.h>

namespace
{
/** @brief Convert integer-keyed JointValues to string-keyed map using the environment's joint names */
std::unordered_map<std::string, double> toStringJointValues(
    const tesseract::scene_graph::SceneState::JointValues& id_map,
    const std::vector<std::string>& joint_names)
{
  std::unordered_map<std::string, double> result;
  for (const auto& name : joint_names)
  {
    auto it = id_map.find(tesseract::common::JointId::fromName(name));
    if (it != id_map.end())
      result[name] = it->second;
  }
  return result;
}
}  // anonymous namespace

namespace tesseract_monitoring
{
CurrentStateMonitor::CurrentStateMonitor(const tesseract::environment::Environment::ConstPtr& env)
  : CurrentStateMonitor(env, std::make_shared<rclcpp::Node>("current_state_monitor"))
{
}

CurrentStateMonitor::CurrentStateMonitor(const tesseract::environment::Environment::ConstPtr& env,
                                         rclcpp::Node::SharedPtr node)
  : node_(std::move(node))
  , env_(env)
  , env_state_(env->getState())
  , last_environment_revision_(env_->getRevision())
  , state_monitor_started_(false)
  , copy_dynamics_(false)
  , error_(std::numeric_limits<double>::epsilon())
{
}

const tesseract::environment::Environment& CurrentStateMonitor::getEnvironment() const { return *env_; }

CurrentStateMonitor::~CurrentStateMonitor() { stopStateMonitor(); }

tesseract::scene_graph::SceneState CurrentStateMonitor::getCurrentState() const
{
  std::scoped_lock slock(state_update_lock_);
  return env_state_;
}

rclcpp::Time CurrentStateMonitor::getCurrentStateTime() const
{
  std::scoped_lock slock(state_update_lock_);
  return current_state_time_;
}

std::pair<tesseract::scene_graph::SceneState, rclcpp::Time> CurrentStateMonitor::getCurrentStateAndTime() const
{
  std::scoped_lock slock(state_update_lock_);
  return std::make_pair(env_state_, current_state_time_);
}

std::unordered_map<std::string, double> CurrentStateMonitor::getCurrentStateValues() const
{
  std::scoped_lock slock(state_update_lock_);
  return toStringJointValues(env_state_.joints, env_->getJointNames());
}

void CurrentStateMonitor::addUpdateCallback(const JointStateUpdateCallback& fn)
{
  if (fn)
    update_callbacks_.push_back(fn);
}

void CurrentStateMonitor::clearUpdateCallbacks() { update_callbacks_.clear(); }

void CurrentStateMonitor::startStateMonitor(const std::string& joint_states_topic)
{
  if (!state_monitor_started_ && env_)
  {
    joint_time_.clear();
    if (joint_states_topic.empty())
    {
      RCLCPP_ERROR(node_->get_logger(), "The joint states topic cannot be an empty string");
      return;
    }

    joint_state_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        joint_states_topic,
        rclcpp::QoS(20),
        std::bind(&CurrentStateMonitor::jointStateCallback, this, std::placeholders::_1));  // NOLINT

    state_monitor_started_ = true;
    monitor_start_time_ = node_->now();
    RCLCPP_DEBUG(node_->get_logger(), "Listening to joint states on topic '%s'", joint_states_topic.c_str());
  }
}

bool CurrentStateMonitor::isActive() const { return state_monitor_started_; }

void CurrentStateMonitor::stopStateMonitor()
{
  if (state_monitor_started_)
  {
    joint_state_subscriber_.reset();
    RCLCPP_DEBUG(node_->get_logger(), "No longer listening to joint states");
    state_monitor_started_ = false;
  }
}

std::string CurrentStateMonitor::getMonitoredTopic() const
{
  if (joint_state_subscriber_)
    return joint_state_subscriber_->get_topic_name();

  return "";
}

bool CurrentStateMonitor::isPassiveOrMimicDOF(const std::string& dof) const
{
  const auto& active_joints = env_->getActiveJointNames();
  auto passive = (std::find(active_joints.begin(), active_joints.end(), dof) == active_joints.end());
  auto mimic = env_->getJoint(dof)->mimic != nullptr;

  return passive || mimic;
}

bool CurrentStateMonitor::haveCompleteState() const
{
  bool result = true;
  std::scoped_lock slock(state_update_lock_);
  for (const auto& name : env_->getJointNames())
    if (env_state_.joints.count(tesseract::common::JointId::fromName(name)) != 0)
      if (joint_time_.find(name) == joint_time_.end())
      {
        if (!isPassiveOrMimicDOF(name))
        {
          RCLCPP_DEBUG(node_->get_logger(), "Joint variable '%s' has never been updated", name.c_str());
          result = false;
        }
      }
  return result;
}

bool CurrentStateMonitor::haveCompleteState(std::vector<std::string>& missing_joints) const
{
  bool result = true;
  std::scoped_lock slock(state_update_lock_);
  for (const auto& name : env_->getJointNames())
    if (env_state_.joints.count(tesseract::common::JointId::fromName(name)) != 0)
      if (joint_time_.find(name) == joint_time_.end())
        if (!isPassiveOrMimicDOF(name))
        {
          RCLCPP_DEBUG(node_->get_logger(), "Joint variable '%s' has never been updated", name.c_str());
          missing_joints.push_back(name);
          result = false;
        }
  return result;
}

bool CurrentStateMonitor::haveCompleteState(const rclcpp::Duration& age) const
{
  bool result = true;
  rclcpp::Time now = node_->now();
  rclcpp::Time old = now - age;
  std::scoped_lock slock(state_update_lock_);
  for (const auto& name : env_->getJointNames())
  {
    if (env_state_.joints.count(tesseract::common::JointId::fromName(name)) == 0)
      continue;
    if (isPassiveOrMimicDOF(name))
      continue;
    auto it = joint_time_.find(name);
    if (it == joint_time_.end())
    {
      RCLCPP_DEBUG(node_->get_logger(), "Joint variable '%s' has never been updated", name.c_str());
      result = false;
    }
    else if (it->second < old)
    {
      RCLCPP_DEBUG(node_->get_logger(),
                   "Joint variable '%s' was last updated %0.3lf seconds ago (older than the allowed %0.3lf seconds)",
                   name.c_str(),
                   (now - it->second).seconds(),
                   age.seconds());
      result = false;
    }
  }
  return result;
}

bool CurrentStateMonitor::haveCompleteState(const rclcpp::Duration& age, std::vector<std::string>& missing_states) const
{
  bool result = true;
  rclcpp::Time now = node_->now();
  rclcpp::Time old = now - age;
  std::scoped_lock slock(state_update_lock_);

  for (const auto& name : env_->getJointNames())
  {
    if (env_state_.joints.count(tesseract::common::JointId::fromName(name)) == 0)
      continue;
    if (isPassiveOrMimicDOF(name))
      continue;
    auto it = joint_time_.find(name);
    if (it == joint_time_.end())
    {
      RCLCPP_DEBUG(node_->get_logger(), "Joint variable '%s' has never been updated", name.c_str());
      missing_states.push_back(name);
      result = false;
    }
    else if (it->second < old)
    {
      RCLCPP_DEBUG(node_->get_logger(),
                   "Joint variable '%s' was last updated %0.3lf seconds ago (older than the allowed %0.3lf seconds)",
                   name.c_str(),
                   (now - it->second).seconds(),
                   age.seconds());
      missing_states.push_back(name);
      result = false;
    }
  }
  return result;
}

bool CurrentStateMonitor::waitForCurrentState(const rclcpp::Time& t, double wait_time) const
{
  rclcpp::Time start = rclcpp::Clock{ RCL_STEADY_TIME }.now();
  rclcpp::Duration elapsed(0, 0);
  rclcpp::Duration timeout = rclcpp::Duration::from_seconds(wait_time);

  std::unique_lock slock(state_update_lock_);
  while (current_state_time_ < t)
  {
    state_update_condition_.wait_for(slock, std::chrono::nanoseconds((timeout - elapsed).nanoseconds()));
    elapsed = rclcpp::Clock{ RCL_STEADY_TIME }.now() - start;
    if (elapsed > timeout)
      return false;
  }
  return true;
}

bool CurrentStateMonitor::waitForCompleteState(double wait_time) const
{
  double slept_time = 0.0;
  double sleep_step_s = std::min(0.05, wait_time / 10.0);

  while (!haveCompleteState() && slept_time < wait_time)
  {
    rclcpp::sleep_for(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(sleep_step_s)));
    slept_time += sleep_step_s;
  }
  return haveCompleteState();
}

bool CurrentStateMonitor::waitForCompleteState(const std::string& manip, double wait_time) const
{
  if (waitForCompleteState(wait_time))
    return true;
  bool ok = true;

  // check to see if we have a fully known state for the joints we want to
  // record
  std::vector<std::string> missing_joints;
  if (!haveCompleteState(missing_joints))
  {
    tesseract::kinematics::JointGroup::ConstPtr jmg = env_->getJointGroup(manip);
    if (jmg)
    {
      std::set<std::string> mj;
      mj.insert(missing_joints.begin(), missing_joints.end());
      const std::vector<std::string>& names = jmg->getJointNames();
      for (std::size_t i = 0; ok && i < names.size(); ++i)
        if (mj.find(names[i]) != mj.end())
          ok = false;
    }
    else
      ok = false;
  }
  return ok;
}

void CurrentStateMonitor::jointStateCallback(const sensor_msgs::msg::JointState::ConstSharedPtr joint_state)  // NOLINT
{
  if (!env_->isInitialized())
    return;

  if (joint_state->name.size() != joint_state->position.size())
  {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(),
                          *node_->get_clock(),
                          rclcpp::Duration::from_seconds(1).nanoseconds(),
                          "State monitor received invalid joint state (number "
                          "of joint names does not match number of "
                          "positions)");
    return;
  }
  bool update = false;

  {
    std::scoped_lock slock(state_update_lock_);
    auto lock = env_->lockRead();
    // read the received values, and update their time stamps
    current_state_time_ = rclcpp::Time(joint_state->header.stamp);
    if (last_environment_revision_ != env_->getRevision())
    {
      env_state_ = env_->getState();
      last_environment_revision_ = env_->getRevision();
    }

    for (unsigned i = 0; i < joint_state->name.size(); ++i)
    {
      auto jid = tesseract::common::JointId::fromName(joint_state->name[i]);
      auto it = env_state_.joints.find(jid);
      if (it != env_state_.joints.end())
      {
        double diff = it->second - joint_state->position[i];
        if (std::fabs(diff) > std::numeric_limits<double>::epsilon())
        {
          it->second = joint_state->position[i];
          update = true;
        }
        joint_time_[joint_state->name[i]] = joint_state->header.stamp;
      }
    }

    if (update)
      env_state_ = env_->getState(toStringJointValues(env_state_.joints, env_->getJointNames()));
  }

  // callbacks, if needed
  if (update)
    for (auto& update_callback : update_callbacks_)
      update_callback(joint_state);

  // notify waitForCurrentState() *after* potential update callbacks
  state_update_condition_.notify_all();
}
}  // namespace tesseract_monitoring
