#ifndef TESSERACT_RVIZ_STATE_MONITORING_H
#define TESSERACT_RVIZ_STATE_MONITORING_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <rviz_common/display.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#ifndef Q_MOC_RUN
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_environment/environment.h>
#endif

#include <tesseract_rviz/render_tools/visualization_widget.h>

namespace rviz_common::properties
{
class Property;
class RosTopicProperty;
}  // namespace rviz_common::properties

namespace tesseract_rviz
{
class JointStateMonitorWidget : public QObject
{
  Q_OBJECT

public:
  using Ptr = std::shared_ptr<JointStateMonitorWidget>;
  using ConstPtr = std::shared_ptr<const JointStateMonitorWidget>;

  JointStateMonitorWidget(rviz_common::properties::Property* widget, rviz_common::Display* display);

  virtual ~JointStateMonitorWidget();

  void onInitialize(VisualizationWidget::Ptr visualization,
                    tesseract_environment::Environment::Ptr env,
                    rviz_common::DisplayContext* context,
                    rclcpp::Node::SharedPtr update_node);

  void onEnable();
  void onDisable();
  void onUpdate();
  void onReset();

private Q_SLOTS:
  void changedJointStateTopic();

protected:
  rviz_common::properties::Property* widget_;
  rviz_common::Display* display_;
  VisualizationWidget::Ptr visualization_;
  tesseract_environment::Environment::Ptr env_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
  bool update_required_;

  rviz_common::properties::Property* main_property_;
  rviz_common::properties::RosTopicProperty* joint_state_topic_property_;

  void newJointStateCallback(const sensor_msgs::msg::JointState::ConstSharedPtr joint_state);

  bool isUpdateRequired(const sensor_msgs::msg::JointState& joint_state);
};
}  // namespace tesseract_rviz
#endif  // TESSERACT_RVIZ_STATE_MONITORING_H
