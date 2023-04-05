/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <rclcpp/rclcpp.hpp>
#include <tesseract_msgs/msg/environment_state.hpp>
#include <tesseract_msgs/srv/modify_environment.hpp>
#include <tesseract_msgs/msg/environment_command.hpp>
#include <tesseract_msgs/srv/get_environment_changes.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

static const std::string ROBOT_DESCRIPTION = "robot_description";

static rclcpp::Service<tesseract_msgs::srv::ModifyEnvironment>::SharedPtr modify_env_rviz;
static rclcpp::Service<tesseract_msgs::srv::GetEnvironmentChanges>::SharedPtr get_env_changes_rviz;

static rclcpp::Service<tesseract_msgs::srv::ModifyEnvironment>::SharedPtr modify_env_master;
static rclcpp::Service<tesseract_msgs::srv::GetEnvironmentChanges>::SharedPtr get_env_changes_master;

rclcpp::Node::SharedPtr node;

void addSphere(const std::string& name, const std::string& id, unsigned long& revision)
{
  tesseract_msgs::srv::ModifyEnvironment update_env;

  // Create add command
  tesseract_msgs::msg::EnvironmentCommand add_sphere_command;
  add_sphere_command.command = tesseract_msgs::msg::EnvironmentCommand::ADD_LINK;

  // Create the link
  add_sphere_command.add_link.name = name;

  tesseract_msgs::msg::VisualGeometry visual;
  visual.origin.position.x = 0.5;
  visual.origin.position.y = 0;
  visual.origin.position.z = 0.55;
  visual.material.empty = true;

  visual.geometry.type = tesseract_msgs::msg::Geometry::SPHERE;
  visual.geometry.sphere_radius = 0.15;

  add_sphere_command.add_link.visual.push_back(visual);

  tesseract_msgs::msg::CollisionGeometry collision;
  collision.origin.position.x = 0.5;
  collision.origin.position.y = 0;
  collision.origin.position.z = 0.55;
  collision.material.empty = true;

  collision.geometry.type = tesseract_msgs::msg::Geometry::SPHERE;
  collision.geometry.sphere_radius = 0.15;

  add_sphere_command.add_link.collision.push_back(collision);

  // Create the Joint
  add_sphere_command.add_joint.type = tesseract_msgs::msg::Joint::FIXED;
  add_sphere_command.add_joint.name = name + "_joint";
  add_sphere_command.add_joint.parent_link_name = "base_link";
  add_sphere_command.add_joint.child_link_name = name;

  update_env.request.id = id;
  update_env.request.revision = revision;
  update_env.request.commands.push_back(add_sphere_command);

  if (modify_env_rviz.call(update_env))
  {
    RCLCPP_INFO(node->get_logger(), "Sphere added to Environment!");
    revision = update_env.response.revision;
  }
  else
  {
    RCLCPP_INFO(node->get_logger(), "Failed to add sphere to environment");
  }
}

void removeSphere(const std::string& name, const std::string& id, unsigned long& revision)
{
  tesseract_msgs::srv::ModifyEnvironment update_env;

  // Create remove command
  tesseract_msgs::msg::EnvironmentCommand command;
  command.command = tesseract_msgs::msg::EnvironmentCommand::REMOVE_LINK;
  command.remove_link = name;

  update_env.request.id = id;
  update_env.request.revision = revision;
  update_env.request.commands.push_back(command);

  if (modify_env_rviz.call(update_env))
  {
    RCLCPP_INFO(node->get_logger(), "Removed sphere from environment!");
    revision = update_env.response.revision;
  }
  else
  {
    RCLCPP_INFO(node->get_logger(), "Failed to remove sphere from environment");
  }
}

bool updateRViz()
{
  get_env_changes_rviz.waitForExistence();
  // Get the current state of the environment
  tesseract_msgs::srv::GetEnvironmentChanges env_changes;
  env_changes.request.revision = 0;
  if (get_env_changes_rviz.call(env_changes))
  {
    RCLCPP_INFO(node->get_logger(), "Retrieve current environment changes!");
  }
  else
  {
    RCLCPP_INFO(node->get_logger(), "Failed retrieve current environment changes!");
    return false;
  }

  std::string id = env_changes.response.id;
  unsigned long revision = env_changes.response.revision;

  modify_env_rviz.waitForExistence();

  addSphere("sphere_attached", id, revision);

  sleep(10);

  removeSphere("sphere_attached", id, revision);

  return true;
}

bool updateMaster()
{
  get_env_changes_master.waitForExistence();
  // Get the current state of the environment
  tesseract_msgs::srv::GetEnvironmentChanges env_changes;
  env_changes.request.revision = 0;
  if (get_env_changes_master.call(env_changes))
  {
    RCLCPP_INFO(node->get_logger(), "Retrieve current environment changes!");
  }
  else
  {
    RCLCPP_INFO(node->get_logger(), "Failed retrieve current environment changes!");
    return false;
  }

  std::string id = env_changes.response.id;
  unsigned long revision = env_changes.response.revision;

  modify_env_master.waitForExistence();

  addSphere("sphere_attached", id, revision);

  sleep(10);

  removeSphere("sphere_attached", id, revision);

  return true;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  node = std::make_shared<rclcpp::Node>("demo");

  std::thread spinner{ [node]() { rclcpp::spin(node); } };

  // These are used to keep visualization updated
  modify_env_rviz = nh.serviceClient<tesseract_msgs::srv::ModifyEnvironment>("modify_tesseract_rviz", true);
  get_env_changes_rviz =
      nh.serviceClient<tesseract_msgs::srv::GetEnvironmentChanges>("get_tesseract_changes_rviz", true);

  // These are used to keep master version of the environment updated
  modify_env_master = nh.serviceClient<tesseract_msgs::srv::ModifyEnvironment>("modify_tesseract", true);
  get_env_changes_master = nh.serviceClient<tesseract_msgs::srv::GetEnvironmentChanges>("get_tesseract_changes", true);

  if (!updateRViz())
    return -1;

  if (!updateMaster())
    return -1;

  ros::waitForShutdown();

  return 0;
}
