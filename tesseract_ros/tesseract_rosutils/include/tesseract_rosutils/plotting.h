/**
 * @file plotting.h
 * @brief Tesseract ROS Basic plotting functions.
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
#ifndef TESSERACT_ROSUTILS_PLOTTING_H
#define TESSERACT_ROSUTILS_PLOTTING_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp/publisher.hpp>
#include <tesseract_msgs/msg/trajectory.hpp>
#include <tesseract_msgs/msg/tesseract_state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_visualization/visualization.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_environment/core/environment.h>

#include <rclcpp/rclcpp.hpp>

#include <thread>

namespace tesseract_rosutils
{
/** @brief The BasicPlotting class */
class ROSPlotting : public tesseract_visualization::Visualization
{
public:
  ROSPlotting(rclcpp::Node::SharedPtr node, tesseract_environment::Environment::ConstPtr env)
    : node_(node)
    , clock_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME))
    , env_(std::move(env))
  {
    trajectory_pub_ = node->create_publisher<tesseract_msgs::msg::Trajectory>("/trajopt/display_tesseract_trajectory", 1);
    collisions_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("/trajopt/display_collisions", 1);
    arrows_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("/trajopt/display_arrows", 1);
    axes_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("/trajopt/display_axes", 1);
    joint_state_pub_ = node->create_publisher<sensor_msgs::msg::JointState>("/trajectory_preview", 1);
  }

  void plotTrajectory(const std::vector<std::string>& joint_names,
                      const Eigen::Ref<const tesseract_common::TrajArray>& traj) override
  {
    tesseract_msgs::msg::Trajectory msg;

    // Set tesseract state information
    toMsg(msg.tesseract_state, *env_);

    // Set the joint trajectory message
    toMsg(msg.joint_trajectory, *(env_->getCurrentState()), joint_names, traj);

    trajectory_pub_->publish(msg);

    sensor_msgs::msg::JointState joint_state;
    joint_state.name = msg.joint_trajectory.joint_names;
    for (auto point : msg.joint_trajectory.points)
    {
      joint_state.header.stamp = clock_->now();
      joint_state.position = point.positions;
      joint_state_pub_->publish(joint_state);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  void plotContactResults(const std::vector<std::string>& link_names,
                          const tesseract_collision::ContactResultVector& dist_results,
                          const Eigen::Ref<const Eigen::VectorXd>& safety_distances) override
  {
    visualization_msgs::msg::MarkerArray msg;
    for (unsigned i = 0; i < dist_results.size(); ++i)
    {
      const tesseract_collision::ContactResult& dist = dist_results[i];
      const double& safety_distance = safety_distances[i];

      Eigen::Vector4d rgba;
      if (dist.distance < 0)
      {
        rgba << 1.0, 0.0, 0.0, 1.0;
      }
      else if (dist.distance < safety_distance)
      {
        rgba << 1.0, 1.0, 0.0, 1.0;
      }
      else
      {
        rgba << 0.0, 1.0, 0.0, 1.0;
      }

      if (dist.cc_type[0] == tesseract_collision::ContinuousCollisionType::CCType_Between)
      {
        Eigen::Vector4d cc_rgba;
        cc_rgba << 0.0, 0.0, 1.0, 1.0;
        auto marker = getMarkerArrowMsg(dist.transform[0] * dist.nearest_points_local[0],
                                        dist.cc_transform[0] * dist.nearest_points_local[0],
                                        cc_rgba,
                                        0.01);
        msg.markers.push_back(marker);
      }

      if (dist.cc_type[1] == tesseract_collision::ContinuousCollisionType::CCType_Between)
      {
        Eigen::Vector4d cc_rgba;
        cc_rgba << 0.0, 0.0, 0.5, 1.0;
        auto marker = getMarkerArrowMsg(dist.transform[1] * dist.nearest_points_local[1],
                                        dist.cc_transform[1] * dist.nearest_points_local[1],
                                        cc_rgba,
                                        0.01);
        msg.markers.push_back(marker);
      }

      auto it0 = std::find(link_names.begin(), link_names.end(), dist.link_names[0]);
      auto it1 = std::find(link_names.begin(), link_names.end(), dist.link_names[1]);

      if (it0 != link_names.end() && it1 != link_names.end())
      {
        auto marker0 = getMarkerArrowMsg(dist.nearest_points[0], dist.nearest_points[1], rgba, 0.01);
        msg.markers.push_back(marker0);

        auto marker1 = getMarkerArrowMsg(dist.nearest_points[1], dist.nearest_points[0], rgba, 0.01);
        msg.markers.push_back(marker1);
      }
      else if (it0 != link_names.end())
      {
        auto marker = getMarkerArrowMsg(dist.nearest_points[1], dist.nearest_points[0], rgba, 0.01);
        msg.markers.push_back(marker);
      }
      else
      {
        auto marker = getMarkerArrowMsg(dist.nearest_points[0], dist.nearest_points[1], rgba, 0.01);
        msg.markers.push_back(marker);
      }
    }

    if (dist_results.size() > 0)
    {
      collisions_pub_->publish(msg);
    }
  }

  void plotArrow(const Eigen::Ref<const Eigen::Vector3d>& pt1,
                 const Eigen::Ref<const Eigen::Vector3d>& pt2,
                 const Eigen::Ref<const Eigen::Vector4d>& rgba,
                 double scale) override
  {
    visualization_msgs::msg::MarkerArray msg;
    msg.markers.push_back(getMarkerArrowMsg(pt1, pt2, rgba, scale));
    arrows_pub_->publish(msg);
  }

  void plotAxis(const Eigen::Isometry3d& axis, double scale) override
  {
    visualization_msgs::msg::MarkerArray msg;
    Eigen::Vector3d x_axis = axis.matrix().block<3, 1>(0, 0);
    Eigen::Vector3d y_axis = axis.matrix().block<3, 1>(0, 1);
    Eigen::Vector3d z_axis = axis.matrix().block<3, 1>(0, 2);
    Eigen::Vector3d position = axis.matrix().block<3, 1>(0, 3);

    msg.markers.push_back(getMarkerCylinderMsg(position, position + x_axis, Eigen::Vector4d(1, 0, 0, 1), scale));
    msg.markers.push_back(getMarkerCylinderMsg(position, position + y_axis, Eigen::Vector4d(0, 1, 0, 1), scale));
    msg.markers.push_back(getMarkerCylinderMsg(position, position + z_axis, Eigen::Vector4d(0, 0, 1, 1), scale));
    axes_pub_->publish(msg);
  }

  void clear() override
  {
    // Remove old arrows
    marker_counter_ = 0;
    visualization_msgs::msg::MarkerArray msg;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = env_->getSceneGraph()->getRoot();
    marker.header.stamp = node_->now();
    marker.ns = "trajopt";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    msg.markers.push_back(marker);
    collisions_pub_->publish(msg);
    arrows_pub_->publish(msg);
    axes_pub_->publish(msg);

    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }

  void waitForInput() override
  {
//    RCLCPP_ERROR(node_->get_logger(), "Waiting 5 seconds while you examine things...");
//    std::this_thread::sleep_for(std::chrono::seconds(5));
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Clock::SharedPtr clock_;
  tesseract_environment::Environment::ConstPtr env_;                                  /**< The Env */
  int marker_counter_;                                                                /**< Counter when plotting */
//  rclcpp::Publisher scene_pub_;                                                     /**< Scene publisher */  // TODO: Unused?
  rclcpp::Publisher<tesseract_msgs::msg::Trajectory>::SharedPtr trajectory_pub_;      /**< Trajectory publisher */
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr collisions_pub_; /**< Collision Data publisher */
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr arrows_pub_;     /**< Used for publishing arrow markers */
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr axes_pub_;       /**< Used for publishing axis markers */
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  visualization_msgs::msg::Marker getMarkerArrowMsg(const Eigen::Ref<const Eigen::Vector3d>& pt1,
                                                    const Eigen::Ref<const Eigen::Vector3d>& pt2,
                                                    const Eigen::Ref<const Eigen::Vector4d>& rgba,
                                                    double scale)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = env_->getSceneGraph()->getRoot();
    marker.header.stamp = node_->now();
    marker.ns = "trajopt";
    marker.id = ++marker_counter_;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    Eigen::Vector3d x, y, z;
    x = (pt2 - pt1).normalized();
    marker.pose.position.x = pt1(0);
    marker.pose.position.y = pt1(1);
    marker.pose.position.z = pt1(2);

    y = x.unitOrthogonal();
    z = (x.cross(y)).normalized();
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

    marker.scale.x = std::abs((pt2 - pt1).norm());
    marker.scale.y = scale;
    marker.scale.z = scale;

    marker.color.r = static_cast<float>(rgba(0));
    marker.color.g = static_cast<float>(rgba(1));
    marker.color.b = static_cast<float>(rgba(2));
    marker.color.a = static_cast<float>(rgba(3));

    return marker;
  }

  visualization_msgs::msg::Marker getMarkerCylinderMsg(const Eigen::Ref<const Eigen::Vector3d>& pt1,
                                                       const Eigen::Ref<const Eigen::Vector3d>& pt2,
                                                       const Eigen::Ref<const Eigen::Vector4d>& rgba,
                                                       double scale)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = env_->getSceneGraph()->getRoot();
    marker.header.stamp = node_->now();
    marker.ns = "trajopt";
    marker.id = ++marker_counter_;
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
};
typedef std::shared_ptr<ROSPlotting> ROSPlottingPtr;
typedef std::shared_ptr<const ROSPlotting> ROSPlottingConstPtr;
}  // namespace tesseract_rosutils

#endif
