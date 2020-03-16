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
#include <tesseract/tesseract.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#endif

#include <tesseract_rviz/render_tools/visualization_widget.h>
#include <tesseract_rviz/render_tools/trajectory_panel.h>

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
  using SharedPtr = std::shared_ptr<TrajectoryMonitorWidget>;
  using ConstSharedPtr = std::shared_ptr<const TrajectoryMonitorWidget>;

  TrajectoryMonitorWidget(rviz_common::properties::Property* widget, rviz_common::Display* display);

  virtual ~TrajectoryMonitorWidget();

  void onInitialize(VisualizationWidget::SharedPtr visualization,
                    tesseract::Tesseract::Ptr tesseract,
                    rviz_common::DisplayContext* context,
                    rclcpp::Node::SharedPtr update_node);

  void onEnable();
  void onDisable();
  void onUpdate(float wall_dt);
  void onReset();
  void onNameChange(const QString& name);

  void dropTrajectory();

public Q_SLOTS:
  void interruptCurrentDisplay();

private Q_SLOTS:
  void changedDisplayMode();
  void changedTrailStepSize();
  void changedTrajectoryTopic();
  void changedStateDisplayTime();
  void trajectorySliderPanelVisibilityChange(bool enable);

protected:
  void incomingDisplayTrajectory(const tesseract_msgs::msg::Trajectory::ConstSharedPtr msg);
  float getStateDisplayTime();
  void clearTrajectoryTrail();
  void createTrajectoryTrail();

  rviz_common::properties::Property* widget_;
  rviz_common::Display* display_;
  rviz_common::DisplayContext* context_;
  VisualizationWidget::SharedPtr visualization_;
  tesseract::Tesseract::Ptr tesseract_;
  rclcpp::Node::SharedPtr node_;
  bool cached_visible_; /**< @brief This caches if the trajectory was visible for enable and disble calls */

  tesseract_msgs::msg::Trajectory::SharedPtr displaying_trajectory_message_;
  tesseract_msgs::msg::Trajectory::SharedPtr trajectory_message_to_display_;

  rclcpp::Subscription<tesseract_msgs::msg::Trajectory>::SharedPtr trajectory_topic_sub_;
  boost::mutex update_trajectory_message_;

  // Pointers from parent display that we save
  bool animating_path_;
  bool drop_displaying_trajectory_;
  int current_state_;
  float current_state_time_;
  TrajectoryPanel* trajectory_slider_panel_;
  rviz_common::PanelDockWidget* trajectory_slider_dock_panel_;
  int previous_display_mode_;
  size_t num_trajectory_waypoints_;

  // Properties
  rviz_common::properties::Property* main_property_;
  rviz_common::properties::EditableEnumProperty* state_display_time_property_;
  rviz_common::properties::RosTopicProperty* trajectory_topic_property_;
  rviz_common::properties::EnumProperty* display_mode_property_;
  rviz_common::properties::BoolProperty* interrupt_display_property_;
  rviz_common::properties::IntProperty* trail_step_size_property_;
};

}  // namespace tesseract_rviz

#endif
