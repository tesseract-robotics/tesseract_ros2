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
#include <tesseract/tesseract.h>
#endif

#include <tesseract_rviz/render_tools/visualization_widget.h>

namespace rviz_common::properties
{
class Property;
class RosTopicProperty;
}  // namespace rviz

namespace tesseract_rviz
{
class JointStateMonitorWidget : public QObject
{
  Q_OBJECT

public:
  using SharedPtr = std::shared_ptr<JointStateMonitorWidget>;
  using ConstSharedPtr = std::shared_ptr<const JointStateMonitorWidget>;

  JointStateMonitorWidget(rviz_common::properties::Property* widget, rviz_common::Display* display);

  virtual ~JointStateMonitorWidget();

  void onInitialize(VisualizationWidget::SharedPtr visualization,
                    tesseract::Tesseract::Ptr tesseract,
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
  VisualizationWidget::SharedPtr visualization_;
  tesseract::Tesseract::Ptr tesseract_;
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
