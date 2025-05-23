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
#include <boost/uuid/random_generator.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tesseract_msgs/msg/trajectory.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_visualization/visualization.h>
#include <tesseract_visualization/markers/arrow_marker.h>
#include <tesseract_visualization/markers/contact_results_marker.h>
#include <tesseract_command_language/poly/instruction_poly.h>

#include <tesseract_environment/commands.h>
#include <tesseract_environment/fwd.h>

namespace tesseract_rosutils
{
/** @brief The BasicPlotting class */
class ROSPlotting : public tesseract_visualization::Visualization
{
public:
  ROSPlotting(std::string root_link = "world", std::string topic_namespace = "tesseract");

  ~ROSPlotting() override;
  ROSPlotting(const ROSPlotting&) = default;
  ROSPlotting& operator=(const ROSPlotting&) = default;
  ROSPlotting(ROSPlotting&&) = default;
  ROSPlotting& operator=(ROSPlotting&&) = default;

  bool isConnected() const override;

  void waitForConnection(long seconds = 0) const override;

  void plotEnvironment(const tesseract_environment::Environment& env, std::string ns = "") override;

  void plotEnvironmentState(const tesseract_scene_graph::SceneState& state, std::string ns = "") override;

  void plotTrajectory(const tesseract_common::JointTrajectory& traj,
                      const tesseract_scene_graph::StateSolver& state_solver,
                      std::string ns = "") override;

  void plotTrajectory(const tesseract_common::JointTrajectory& traj, std::string ns = "", std::string description = "");

  void plotTrajectory(const tesseract_msgs::msg::Trajectory& traj);

  void plotTrajectory(const tesseract_environment::Environment& env,
                      const tesseract_planning::InstructionPoly& instruction,
                      std::string ns = "");

  void plotTrajectories(const tesseract_environment::Environment& env,
                        const std::vector<tesseract_planning::InstructionPoly>& instructions,
                        std::string ns = "",
                        const std::string& description = "",
                        const boost::uuids::uuid& uuid = boost::uuids::random_generator()());

  void plotTrajectory(const std::vector<std::shared_ptr<const tesseract_environment::Command>>& cmds,
                      const tesseract_planning::InstructionPoly& instruction,
                      std::string ns = "");

  void plotTrajectories(const tesseract_environment::Commands& cmds,
                        const std::vector<tesseract_planning::InstructionPoly>& instructions,
                        std::string ns = "",
                        const std::string& description = "",
                        const boost::uuids::uuid& uuid = boost::uuids::random_generator()());

  void plotToolpath(const tesseract_common::Toolpath& toolpath, std::string ns = "");

  void plotToolpath(const tesseract_environment::Environment& env,
                    const tesseract_planning::InstructionPoly& instruction,
                    std::string ns = "");

  void plotMarker(const tesseract_visualization::Marker& marker, std::string ns = "") override;

  void plotMarkers(const std::vector<tesseract_visualization::Marker::Ptr>& markers, std::string ns = "") override;

  void clear(std::string ns = "") override;

  void waitForInput(std::string message = "Hit enter key to continue!") override;

  const std::string& getRootLink() const;

  void resetCounter();

  static visualization_msgs::msg::MarkerArray getMarkerAxisMsg(int& id_counter,
                                                               const std::string& frame_id,
                                                               const std::string& ns,
                                                               const rclcpp::Time& time_stamp,
                                                               const Eigen::Isometry3d& axis,
                                                               const Eigen::Vector3d& scale);

  static visualization_msgs::msg::Marker getMarkerArrowMsg(int& id_counter,
                                                           const std::string& frame_id,
                                                           const std::string& ns,
                                                           const rclcpp::Time& time_stamp,
                                                           const tesseract_visualization::ArrowMarker& marker);

  static visualization_msgs::msg::Marker getMarkerCylinderMsg(int& id_counter,
                                                              const std::string& frame_id,
                                                              const std::string& ns,
                                                              const rclcpp::Time& time_stamp,
                                                              const Eigen::Ref<const Eigen::Vector3d>& pt1,
                                                              const Eigen::Ref<const Eigen::Vector3d>& pt2,
                                                              const Eigen::Ref<const Eigen::Vector4d>& rgba,
                                                              double scale);

  static visualization_msgs::msg::MarkerArray
  getContactResultsMarkerArrayMsg(int& id_counter,
                                  const std::string& frame_id,
                                  const std::string& ns,
                                  const rclcpp::Time& time_stamp,
                                  const tesseract_visualization::ContactResultsMarker& marker);

private:
  std::string root_link_;       /**< Root link of markers */
  std::string topic_namespace_; /**< Namespace used when publishing markers */
  int marker_counter_{ 0 };     /**< Counter when plotting */
  rclcpp::Node::SharedPtr internal_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr internal_node_executor_;
  std::shared_ptr<std::thread> internal_node_spinner_;
  rclcpp::Publisher<tesseract_msgs::msg::SceneGraph>::SharedPtr scene_pub_;           /**< Scene publisher */
  rclcpp::Publisher<tesseract_msgs::msg::Trajectory>::SharedPtr trajectory_pub_;      /**< Trajectory publisher */
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr collisions_pub_; /**< Collision Data publisher */
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr arrows_pub_; /**< Used for publishing arrow markers
                                                                                   */
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr axes_pub_; /**< Used for publishing axis markers */
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tool_path_pub_; /**< Used for publishing tool path
                                                                                        markers */
};
using ROSPlottingPtr = std::shared_ptr<ROSPlotting>;
using ROSPlottingConstPtr = std::shared_ptr<const ROSPlotting>;
}  // namespace tesseract_rosutils

#endif
