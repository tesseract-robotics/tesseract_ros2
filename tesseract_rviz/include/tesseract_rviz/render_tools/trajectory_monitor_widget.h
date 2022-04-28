/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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

/* Author: Dave Coleman */

#ifndef TESSERACT_RVIZ_TRAJECTORY_MONITOR_WIDGET
#define TESSERACT_RVIZ_TRAJECTORY_MONITOR_WIDGET

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <rviz_common/display.hpp>
#include <rviz_common/panel_dock_widget.hpp>
#include <boost/thread/mutex.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#ifndef Q_MOC_RUN
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <rclcpp/rclcpp.hpp>
#include <tesseract_msgs/msg/trajectory.hpp>
#include <tesseract_environment/environment.h>
#include <tesseract_common/types.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#endif

#include <tesseract_rviz/render_tools/visualize_trajectory_widget.h>

namespace rviz_rendering
{
class Shape;
class MovableText;
}

namespace rviz_common::properties
{
class Property;
class IntProperty;
class StringProperty;
class BoolProperty;
class FloatProperty;
class RosTopicProperty;
class EditableEnumProperty;
class EnumProperty;
class ColorProperty;
}  // namespace rviz

namespace tesseract_rviz
{
class TrajectoryMonitorWidget : public QObject
{
  Q_OBJECT

public:
  using Ptr = std::shared_ptr<TrajectoryMonitorWidget>;
  using ConstPtr = std::shared_ptr<const TrajectoryMonitorWidget>;

  TrajectoryMonitorWidget(rviz_common::properties::Property* widget, rviz_common::Display* display);

  virtual ~TrajectoryMonitorWidget();

  void onInitialize(VisualizationWidget::Ptr visualization,
                    tesseract_environment::Environment::Ptr env,
                    rviz_common::DisplayContext* context,
                    rclcpp::Node::SharedPtr update_node);

  void onEnable();
  void onDisable();
  void onReset();
  void onUpdate(float wall_dt);
  void onNameChange(const QString& name);

private Q_SLOTS:
  void changedTrajectoryTopic();

public:
  void incomingDisplayTrajectory(const tesseract_msgs::msg::Trajectory::ConstSharedPtr msg);


protected:
  rviz_common::properties::Property* widget_;
  rviz_common::Display* display_;
  rviz_common::DisplayContext* context_;
 
  rclcpp::Node::SharedPtr node_;
  VisualizeTrajectoryWidget::Ptr visualize_trajectory_widget_;

  /**
   * @brief These are the command that were pushed to motion planning
   * @details Visualization only supports moving objects around in a trajectory visualization.
   * If it includes commands which delete or adds links visualization may not be correct.
   * @todo Need to maintain two environments one which tracks the monitored environment and then
   * when new request are recieved it clones this environment an applys the commands througout the
   * trajectory widget.
   */
  tesseract_environment::Commands trajectory_env_commands_;

  rclcpp::Subscription<tesseract_msgs::msg::Trajectory>::SharedPtr trajectory_topic_sub_;
  boost::mutex update_trajectory_message_;

  // Properties
  rviz_common::properties::Property* main_property_;
  rviz_common::properties::RosTopicProperty* trajectory_topic_property_;
};

}  // namespace tesseract_rviz

#endif
