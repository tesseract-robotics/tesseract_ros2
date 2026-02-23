/**
 * @file plotting.cpp
 * @brief Tesseract ROS plotting functions.
 *
 * @author Levi Armstrong
 * @date April 15, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2013, Southwest Research Institute
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
#include <thread>
#include <Eigen/Geometry>
#include <boost/uuid/uuid_io.hpp>
#include <rclcpp/rclcpp.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>

#include <tesseract_command_language/cereal_serialization.h>
#include <tesseract_command_language/utils.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_common/serialization.h>

#include <tesseract_environment/environment.h>
#include <tesseract_environment/command.h>
#include <tesseract_environment/commands.h>

#include <tesseract_visualization/markers/arrow_marker.h>
#include <tesseract_visualization/markers/axis_marker.h>
#include <tesseract_visualization/markers/contact_results_marker.h>
#include <tesseract_visualization/markers/toolpath_marker.h>

namespace tesseract_rosutils
{
static constexpr const char* NODE_ID = "tesseract_rosutils_plotting";

ROSPlotting::ROSPlotting(std::string root_link, std::string topic_namespace)
  : root_link_(std::move(root_link)), topic_namespace_(std::move(topic_namespace))
{
  // Create a unique name for the node
  char node_name[45];                                                                         // NOLINT
  snprintf(node_name, sizeof(node_name), "%s_%zx", NODE_ID, reinterpret_cast<size_t>(this));  // NOLINT
  internal_node_ = std::make_shared<rclcpp::Node>(node_name);
  trajectory_pub_ = internal_node_->create_publisher<tesseract_msgs::msg::Trajectory>(topic_namespace_ + "/display_"
                                                                                                         "tesseract_"
                                                                                                         "trajectory",
                                                                                      rclcpp::QoS(20));
  collisions_pub_ = internal_node_->create_publisher<visualization_msgs::msg::MarkerArray>(topic_namespace_ + "/display"
                                                                                                              "_collisi"
                                                                                                              "ons",
                                                                                           rclcpp::QoS(20));
  arrows_pub_ = internal_node_->create_publisher<visualization_msgs::msg::MarkerArray>(topic_namespace_ + "/display_"
                                                                                                          "arrows",
                                                                                       rclcpp::QoS(20));
  axes_pub_ = internal_node_->create_publisher<visualization_msgs::msg::MarkerArray>(topic_namespace_ + "/display_axes",
                                                                                     rclcpp::QoS(20));
  tool_path_pub_ = internal_node_->create_publisher<visualization_msgs::msg::MarkerArray>(topic_namespace_ + "/display_"
                                                                                                             "tool_"
                                                                                                             "path",
                                                                                          rclcpp::QoS(20));

  internal_node_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  internal_node_spinner_ = std::make_shared<std::thread>([this]() {
    internal_node_executor_->add_node(internal_node_);
    internal_node_executor_->spin();
    internal_node_executor_->remove_node(internal_node_);
  });
}

ROSPlotting::~ROSPlotting()
{
  internal_node_executor_->cancel();
  if (internal_node_spinner_->joinable())
    internal_node_spinner_->join();
}

bool ROSPlotting::isConnected() const { return true; }

void ROSPlotting::waitForConnection(long seconds) const
{
  const auto start_time = rclcpp::Clock{ RCL_STEADY_TIME }.now();
  const auto wall_timeout = rclcpp::Duration::from_seconds(static_cast<double>(seconds));

  while (rclcpp::ok())
  {
    if (isConnected())
      return;

    if (wall_timeout >= rclcpp::Duration::from_seconds(0))
    {
      const auto current_time = rclcpp::Clock{ RCL_STEADY_TIME }.now();
      if ((current_time - start_time) >= wall_timeout)
        return;
    }

    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.02)));
  }
}

void ROSPlotting::plotEnvironment(const tesseract::environment::Environment& /*env*/, std::string /*ns*/) {}

void ROSPlotting::plotEnvironmentState(const tesseract::scene_graph::SceneState& /*state*/, std::string /*ns*/) {}

void ROSPlotting::plotTrajectory(const tesseract::common::JointTrajectory& traj,
                                 const tesseract::scene_graph::StateSolver& /*state_solver*/,
                                 std::string ns)
{
  plotTrajectory(traj, ns);
}

void ROSPlotting::plotTrajectory(const tesseract::common::JointTrajectory& traj,
                                 std::string ns,
                                 std::string description)
{
  tesseract_msgs::msg::Trajectory msg;
  msg.ns = std::move(ns);
  msg.joint_trajectories_uuid = boost::uuids::to_string(traj.uuid);
  msg.joint_trajectories_description = std::move(description);

  if (!traj.empty())
  {
    // Set the initial state
    for (std::size_t i = 0; i < traj[0].joint_names.size(); ++i)
    {
      tesseract_msgs::msg::StringDoublePair pair;
      pair.first = traj[0].joint_names[i];
      pair.second = traj[0].position[static_cast<Eigen::Index>(i)];
      msg.initial_state.push_back(pair);
    }
  }

  // Set the joint trajectory message
  tesseract_msgs::msg::JointTrajectory traj_msg;
  toMsg(traj_msg, traj);
  msg.joint_trajectories.push_back(traj_msg);

  plotTrajectory(msg);
}

void ROSPlotting::plotTrajectory(const tesseract_msgs::msg::Trajectory& traj) { trajectory_pub_->publish(traj); }

void ROSPlotting::plotTrajectory(const tesseract::environment::Environment& env,
                                 const tesseract::command_language::InstructionPoly& instruction,
                                 std::string ns)
{
  tesseract_msgs::msg::Trajectory msg;
  msg.ns = std::move(ns);

  // Set tesseract state information
  toMsg(msg.environment, env);

  // Set the initial state
  tesseract::scene_graph::SceneState initial_state = env.getState();
  tesseract_rosutils::toMsg(msg.initial_state, initial_state.joints);

  assert(instruction.isCompositeInstruction());
  const auto& ci = instruction.as<tesseract::command_language::CompositeInstruction>();
  msg.instructions =
      tesseract::common::Serialization::toArchiveStringXML<tesseract::command_language::CompositeInstruction>(ci);

  plotTrajectory(msg);
}

void ROSPlotting::plotTrajectories(const tesseract::environment::Environment& env,
                                   const std::vector<tesseract::command_language::InstructionPoly>& instructions,
                                   std::string ns,
                                   const std::string& description,
                                   const boost::uuids::uuid& uuid)
{
  tesseract::command_language::CompositeInstruction ci;
  ci.setDescription(description);
  ci.setUUID(uuid);

  // Add all instructions to one composite instruction
  for (const auto& instruction : instructions)
  {
    ci.emplace_back(instruction);
  }

  plotTrajectory(env, ci, std::move(ns));
}

void ROSPlotting::plotTrajectory(const tesseract::environment::Commands& cmds,
                                 const tesseract::command_language::InstructionPoly& instruction,
                                 std::string ns)
{
  tesseract_msgs::msg::Trajectory msg;
  msg.ns = std::move(ns);
  msg.joint_trajectories_uuid = boost::uuids::to_string(instruction.getUUID());

  // Set the commands message
  std::vector<tesseract_msgs::msg::EnvironmentCommand> env_cmds;
  tesseract_rosutils::toMsg(env_cmds, cmds, 0);
  msg.commands = env_cmds;

  // Convert to joint trajectory
  assert(instruction.isCompositeInstruction());
  const auto& ci = instruction.as<tesseract::command_language::CompositeInstruction>();
  msg.joint_trajectories_description = ci.getDescription();
  tesseract::common::JointTrajectory traj = tesseract::command_language::toJointTrajectory(ci);

  if (!traj.empty())
  {
    // Set the initial state
    for (std::size_t i = 0; i < traj[0].joint_names.size(); ++i)
    {
      tesseract_msgs::msg::StringDoublePair pair;
      pair.first = traj[0].joint_names[i];
      pair.second = traj[0].position[static_cast<Eigen::Index>(i)];
      msg.initial_state.push_back(pair);
    }
  }

  // Set the joint trajectory message
  tesseract_msgs::msg::JointTrajectory traj_msg;
  toMsg(traj_msg, traj);
  msg.joint_trajectories.push_back(traj_msg);

  plotTrajectory(msg);
}

void ROSPlotting::plotTrajectories(const tesseract::environment::Commands& cmds,
                                   const std::vector<tesseract::command_language::InstructionPoly>& instructions,
                                   std::string ns,
                                   const std::string& description,
                                   const boost::uuids::uuid& uuid)
{
  tesseract_msgs::msg::Trajectory msg;
  msg.ns = std::move(ns);
  msg.joint_trajectories_description = description;
  msg.joint_trajectories_uuid = boost::uuids::to_string(uuid);

  // Set the commands message
  std::vector<tesseract_msgs::msg::EnvironmentCommand> env_cmds;
  tesseract_rosutils::toMsg(env_cmds, cmds, 0);
  msg.commands = env_cmds;

  bool first = true;
  for (const auto& instruction : instructions)
  {
    // Convert to joint trajectory
    assert(instruction.isCompositeInstruction());
    const auto& ci = instruction.as<tesseract::command_language::CompositeInstruction>();
    tesseract::common::JointTrajectory traj = tesseract::command_language::toJointTrajectory(ci);

    // Set the joint trajectory message
    tesseract_msgs::msg::JointTrajectory traj_msg;
    toMsg(traj_msg, traj);
    msg.joint_trajectories.push_back(traj_msg);

    if (first)
    {
      if (!traj.empty())
      {
        // Set the initial state
        for (std::size_t i = 0; i < traj[0].joint_names.size(); ++i)
        {
          tesseract_msgs::msg::StringDoublePair pair;
          pair.first = traj[0].joint_names[i];
          pair.second = traj[0].position[static_cast<Eigen::Index>(i)];
          msg.initial_state.push_back(pair);
        }
        first = false;
      }
    }
  }

  plotTrajectory(msg);
}

void ROSPlotting::plotMarker(const tesseract::visualization::Marker& marker, std::string ns)
{
  switch (marker.getType())
  {
    case static_cast<int>(tesseract::visualization::MarkerType::ARROW):
    {
      const auto& m = dynamic_cast<const tesseract::visualization::ArrowMarker&>(marker);
      visualization_msgs::msg::MarkerArray msg;
      auto arrow_marker_msg =
          getMarkerArrowMsg(marker_counter_, root_link_, topic_namespace_, internal_node_->now(), m);
      msg.markers.push_back(arrow_marker_msg);
      arrows_pub_->publish(msg);
      break;
    }
    case static_cast<int>(tesseract::visualization::MarkerType::AXIS):
    {
      const auto& m = dynamic_cast<const tesseract::visualization::AxisMarker&>(marker);
      visualization_msgs::msg::MarkerArray msg =
          getMarkerAxisMsg(marker_counter_, root_link_, topic_namespace_, internal_node_->now(), m.axis, m.getScale());
      axes_pub_->publish(msg);
      break;
    }
    case static_cast<int>(tesseract::visualization::MarkerType::TOOLPATH):
    {
      const auto& m = dynamic_cast<const tesseract::visualization::ToolpathMarker&>(marker);
      std::string prefix_ns = topic_namespace_;
      if (!ns.empty())
        prefix_ns = topic_namespace_ + "/" + ns;

      visualization_msgs::msg::MarkerArray msg;
      long cnt = 0;
      auto time = internal_node_->now();
      for (const auto& s : m.toolpath)
      {
        std::string segment_ns = prefix_ns + "/segment_" + std::to_string(cnt++) + "/poses";
        for (const auto& p : s)
        {
          visualization_msgs::msg::MarkerArray msg_pose =
              getMarkerAxisMsg(marker_counter_, root_link_, segment_ns, time, p, m.scale);
          msg.markers.insert(msg.markers.end(), msg_pose.markers.begin(), msg_pose.markers.end());
        }
      }
      tool_path_pub_->publish(msg);
      break;
    }
    case static_cast<int>(tesseract::visualization::MarkerType::CONTACT_RESULTS):
    {
      const auto& m = dynamic_cast<const tesseract::visualization::ContactResultsMarker&>(marker);
      if (!m.dist_results.empty())
      {
        visualization_msgs::msg::MarkerArray msg =
            getContactResultsMarkerArrayMsg(marker_counter_, root_link_, topic_namespace_, internal_node_->now(), m);
        collisions_pub_->publish(msg);
      }
      break;
    }
    default:
    {
      RCLCPP_ERROR(internal_node_->get_logger(), "ROSPlotting: Unsupported marker type!");
    }
  }
}

void ROSPlotting::plotMarkers(const std::vector<tesseract::visualization::Marker::Ptr>& /*markers*/, std::string /*ns*/)
{
  RCLCPP_ERROR(internal_node_->get_logger(), "ROSPlotting: Plotting vector of markers is currently not implemented!");
}

void ROSPlotting::plotToolpath(const tesseract::common::Toolpath& toolpath, std::string ns)
{
  tesseract::visualization::ToolpathMarker marker(toolpath);
  plotMarker(marker, std::move(ns));
}

void ROSPlotting::plotToolpath(const tesseract::environment::Environment& env,
                               const tesseract::command_language::InstructionPoly& instruction,
                               std::string ns)
{
  tesseract::common::Toolpath toolpath = tesseract::motion_planners::toToolpath(instruction, env);
  plotToolpath(toolpath, std::move(ns));
}

visualization_msgs::msg::MarkerArray ROSPlotting::getMarkerAxisMsg(int& id_counter,
                                                                   const std::string& frame_id,
                                                                   const std::string& ns,
                                                                   const rclcpp::Time& time_stamp,
                                                                   const Eigen::Isometry3d& axis,
                                                                   const Eigen::Vector3d& scale)
{
  visualization_msgs::msg::MarkerArray msg;
  Eigen::Vector3d x_axis = axis.matrix().block<3, 1>(0, 0);
  Eigen::Vector3d y_axis = axis.matrix().block<3, 1>(0, 1);
  Eigen::Vector3d z_axis = axis.matrix().block<3, 1>(0, 2);
  Eigen::Vector3d position = axis.matrix().block<3, 1>(0, 3);

  auto marker_msg = getMarkerCylinderMsg(
      id_counter, frame_id, ns, time_stamp, position, position + x_axis, Eigen::Vector4d(1, 0, 0, 1), scale(0));
  msg.markers.push_back(marker_msg);

  marker_msg = getMarkerCylinderMsg(
      id_counter, frame_id, ns, time_stamp, position, position + y_axis, Eigen::Vector4d(0, 1, 0, 1), scale(1));
  msg.markers.push_back(marker_msg);

  marker_msg = getMarkerCylinderMsg(
      id_counter, frame_id, ns, time_stamp, position, position + z_axis, Eigen::Vector4d(0, 0, 1, 1), scale(2));
  msg.markers.push_back(marker_msg);
  return msg;
}

void ROSPlotting::clear(std::string ns)
{
  // Remove old markers
  marker_counter_ = 0;
  visualization_msgs::msg::MarkerArray msg;
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = root_link_;
  marker.header.stamp = rclcpp::Time();
  marker.ns = ns;
  marker.id = 0;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  msg.markers.push_back(marker);
  collisions_pub_->publish(msg);
  arrows_pub_->publish(msg);
  axes_pub_->publish(msg);
  tool_path_pub_->publish(msg);
}

static void waitForInputAsync(const std::string& message)
{
  RCLCPP_ERROR(rclcpp::get_logger(NODE_ID), "%s", message.c_str());
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

void ROSPlotting::waitForInput(std::string /*message*/)
{
  // std::chrono::microseconds timeout(1);
  // std::future<void> future = std::async(std::launch::async, [=]() { waitForInputAsync(message); });
  // while (future.wait_for(timeout) != std::future_status::ready)
  //   rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.1)));
}

const std::string& ROSPlotting::getRootLink() const { return root_link_; }

void ROSPlotting::resetCounter() { marker_counter_ = 0; }

visualization_msgs::msg::Marker ROSPlotting::getMarkerArrowMsg(int& id_counter,
                                                               const std::string& frame_id,
                                                               const std::string& ns,
                                                               const rclcpp::Time& time_stamp,
                                                               const tesseract::visualization::ArrowMarker& marker)
{
  visualization_msgs::msg::Marker marker_msg;
  marker_msg.header.frame_id = frame_id;
  marker_msg.header.stamp = time_stamp;
  marker_msg.ns = ns;
  marker_msg.id = ++id_counter;
  marker_msg.type = visualization_msgs::msg::Marker::ARROW;
  marker_msg.action = visualization_msgs::msg::Marker::ADD;
  marker_msg.pose.position.x = marker.pose.translation().x();
  marker_msg.pose.position.y = marker.pose.translation().y();
  marker_msg.pose.position.z = marker.pose.translation().z();

  Eigen::Quaterniond q(marker.pose.rotation());
  q.normalize();
  marker_msg.pose.orientation.x = q.x();
  marker_msg.pose.orientation.y = q.y();
  marker_msg.pose.orientation.z = q.z();
  marker_msg.pose.orientation.w = q.w();

  marker_msg.scale.x = marker.shaft_length + marker.head_length;
  marker_msg.scale.y = marker.shaft_radius;
  marker_msg.scale.z = marker.shaft_radius;

  marker_msg.color.r = static_cast<float>(marker.material->color(0));
  marker_msg.color.g = static_cast<float>(marker.material->color(1));
  marker_msg.color.b = static_cast<float>(marker.material->color(2));
  marker_msg.color.a = static_cast<float>(marker.material->color(3));

  return marker_msg;
}

visualization_msgs::msg::Marker ROSPlotting::getMarkerCylinderMsg(int& id_counter,
                                                                  const std::string& frame_id,
                                                                  const std::string& ns,
                                                                  const rclcpp::Time& time_stamp,
                                                                  const Eigen::Ref<const Eigen::Vector3d>& pt1,
                                                                  const Eigen::Ref<const Eigen::Vector3d>& pt2,
                                                                  const Eigen::Ref<const Eigen::Vector4d>& rgba,
                                                                  double scale)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = time_stamp;
  marker.ns = ns;
  marker.id = ++id_counter;
  marker.type = visualization_msgs::msg::Marker::CYLINDER;
  marker.action = visualization_msgs::msg::Marker::ADD;

  double length = scale * std::abs((pt2 - pt1).norm());
  Eigen::Vector3d x, y, z, center;
  z = (pt2 - pt1).normalized();
  center = pt1 + (length / 2.0) * z;
  marker.pose.position.x = center(0);
  marker.pose.position.y = center(1);
  marker.pose.position.z = center(2);

  y = z.unitOrthogonal();
  x = (y.cross(z)).normalized();
  Eigen::Matrix3d rot;
  rot.col(0) = x;
  rot.col(1) = y;
  rot.col(2) = z;
  Eigen::Quaterniond q(rot);
  q.normalize();
  marker.pose.orientation.x = q.x();
  marker.pose.orientation.y = q.y();
  marker.pose.orientation.z = q.z();
  marker.pose.orientation.w = q.w();

  marker.scale.x = length / 20.0;
  marker.scale.y = length / 20.0;
  marker.scale.z = length;

  marker.color.r = static_cast<float>(rgba(0));
  marker.color.g = static_cast<float>(rgba(1));
  marker.color.b = static_cast<float>(rgba(2));
  marker.color.a = static_cast<float>(rgba(3));

  return marker;
}

visualization_msgs::msg::MarkerArray
ROSPlotting::getContactResultsMarkerArrayMsg(int& id_counter,
                                             const std::string& frame_id,
                                             const std::string& ns,
                                             const rclcpp::Time& time_stamp,
                                             const tesseract::visualization::ContactResultsMarker& marker)
{
  visualization_msgs::msg::MarkerArray msg;
  for (unsigned i = 0; i < marker.dist_results.size(); ++i)
  {
    const tesseract::collision::ContactResult& dist = marker.dist_results[i];
    double safety_distance{ 0 };
    if (marker.margin_fn != nullptr)
      safety_distance = marker.margin_fn(dist.link_names[0], dist.link_names[1]);
    else
      safety_distance = marker.margin_data.getCollisionMargin(dist.link_names[0], dist.link_names[1]);

    auto base_material = std::make_shared<tesseract::scene_graph::Material>("base_material");
    if (dist.distance < 0)
      base_material->color << 1.0, 0.0, 0.0, 1.0;
    else if (dist.distance < safety_distance)
      base_material->color << 1.0, 1.0, 0.0, 1.0;
    else
      base_material->color << 0.0, 1.0, 0.0, 1.0;

    if (dist.cc_type[0] == tesseract::collision::ContinuousCollisionType::CCType_Between)
    {
      tesseract::visualization::ArrowMarker am(dist.transform[0] * dist.nearest_points_local[0],
                                               dist.cc_transform[0] * dist.nearest_points_local[0]);
      am.material = std::make_shared<tesseract::scene_graph::Material>("cc_material");
      am.material->color << 0.0, 0.0, 1.0, 1.0;
      auto marker = getMarkerArrowMsg(id_counter, frame_id, ns, time_stamp, am);
      msg.markers.push_back(marker);
    }

    if (dist.cc_type[1] == tesseract::collision::ContinuousCollisionType::CCType_Between)
    {
      tesseract::visualization::ArrowMarker am(dist.transform[1] * dist.nearest_points_local[1],
                                               dist.cc_transform[1] * dist.nearest_points_local[1]);
      am.material = std::make_shared<tesseract::scene_graph::Material>("cc_material");
      am.material->color << 0.0, 0.0, 0.5, 1.0;
      auto marker = getMarkerArrowMsg(id_counter, frame_id, ns, time_stamp, am);
      msg.markers.push_back(marker);
    }

    auto it0 = std::find(marker.link_names.begin(), marker.link_names.end(), dist.link_names[0]);
    auto it1 = std::find(marker.link_names.begin(), marker.link_names.end(), dist.link_names[1]);

    if (it0 != marker.link_names.end() && it1 != marker.link_names.end())
    {
      tesseract::visualization::ArrowMarker am1(dist.nearest_points[0], dist.nearest_points[1]);
      am1.material = base_material;
      auto marker0 = getMarkerArrowMsg(id_counter, frame_id, ns, time_stamp, am1);
      msg.markers.push_back(marker0);

      tesseract::visualization::ArrowMarker am2(dist.nearest_points[1], dist.nearest_points[0]);
      am2.material = base_material;
      auto marker1 = getMarkerArrowMsg(id_counter, frame_id, ns, time_stamp, am2);
      msg.markers.push_back(marker1);
    }
    else if (it0 != marker.link_names.end())
    {
      tesseract::visualization::ArrowMarker am(dist.nearest_points[1], dist.nearest_points[0]);
      am.material = base_material;
      auto marker = getMarkerArrowMsg(id_counter, frame_id, ns, time_stamp, am);
      msg.markers.push_back(marker);
    }
    else
    {
      tesseract::visualization::ArrowMarker am(dist.nearest_points[0], dist.nearest_points[1]);
      am.material = base_material;
      auto marker = getMarkerArrowMsg(id_counter, frame_id, ns, time_stamp, am);
      msg.markers.push_back(marker);
    }
  }

  return msg;
}

}  // namespace tesseract_rosutils
