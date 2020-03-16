#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/window_manager_interface.hpp>

#include <tesseract_rosutils/utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_rviz/render_tools/joint_state_monitor_widget.h>

namespace tesseract_rviz
{
JointStateMonitorWidget::JointStateMonitorWidget(rviz_common::properties::Property* widget, rviz_common::Display* display)
  : widget_(widget), display_(display), visualization_(nullptr), tesseract_(nullptr), update_required_(false)
{
  main_property_ = new rviz_common::properties::Property(
      "Joint State Monitor", "", "Monitor a joint state topic and update the visualization", widget_, nullptr, this);

  joint_state_topic_property_ = new rviz_common::properties::RosTopicProperty("Topic",
                                                           "joint_states",
                                                           "sensor_msgs::JointState",
                                                           "The topic on which the sensor_msgs::JointState messages "
                                                           "are received",
                                                           main_property_,
                                                           SLOT(changedJointStateTopic()),
                                                           this);
}

JointStateMonitorWidget::~JointStateMonitorWidget() { /*joint_state_subscriber_.shutdown();*/ }

void JointStateMonitorWidget::onInitialize(VisualizationWidget::SharedPtr visualization,
                                           tesseract::Tesseract::Ptr tesseract,
                                           rviz_common::DisplayContext* context,
                                           rclcpp::Node::SharedPtr update_node)
{
  visualization_ = std::move(visualization);
  tesseract_ = std::move(tesseract);
  node_ = update_node;

  auto rviz_ros_node = context->getRosNodeAbstraction();
  joint_state_topic_property_->initialize(rviz_ros_node);
}

void JointStateMonitorWidget::changedJointStateTopic()
{
  if(joint_state_topic_property_ && joint_state_topic_property_->getStdString() != "")
  {
  joint_state_subscriber_.reset();

  auto current_state_cb = std::bind(&JointStateMonitorWidget::newJointStateCallback, this, std::placeholders::_1);
  joint_state_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_property_->getStdString(), 10, current_state_cb);
  }
  else
    CONSOLE_BRIDGE_logWarn("joint state topic is invalid");
}

void JointStateMonitorWidget::newJointStateCallback(const sensor_msgs::msg::JointState::ConstSharedPtr joint_state_msg)
{
  if (!tesseract_->isInitialized())
    return;

  if (isUpdateRequired(*joint_state_msg))
  {
    CONSOLE_BRIDGE_logWarn("Conversion has not been defined in rosutils yet. Go do that.");
    tesseract_rosutils::processMsg(tesseract_->getEnvironment(), *joint_state_msg);
    update_required_ = true;
  }
}

void JointStateMonitorWidget::onEnable() { changedJointStateTopic(); }

void JointStateMonitorWidget::onDisable() { joint_state_subscriber_.reset(); }

void JointStateMonitorWidget::onUpdate()
{
  if (visualization_ && update_required_ && tesseract_->getEnvironment())
  {
    update_required_ = false;
    visualization_->update(tesseract_->getEnvironment()->getCurrentState()->transforms);
  }
}

void JointStateMonitorWidget::onReset() { changedJointStateTopic(); }

bool JointStateMonitorWidget::isUpdateRequired(const sensor_msgs::msg::JointState& joint_state)
{
  std::unordered_map<std::string, double> joints = tesseract_->getEnvironment()->getCurrentState()->joints;
  for (auto i = 0u; i < joint_state.name.size(); ++i)
    if (std::abs(joints[joint_state.name[i]] - joint_state.position[i]) > 1e-5)
      return true;

  return false;
}

}  // namespace tesseract_rviz
