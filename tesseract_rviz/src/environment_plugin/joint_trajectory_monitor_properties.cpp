#include <tesseract_rviz/environment_plugin/joint_trajectory_monitor_properties.h>
#include <tesseract_rviz/environment_plugin/conversions.h>

#include <tesseract_qt/joint_trajectory/joint_trajectory_widget.h>
#include <tesseract_qt/common/joint_trajectory_set.h>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <tesseract_msgs/msg/trajectory.hpp>
#include <tesseract_rosutils/utils.h>

#include <rviz_common/display.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/panel_dock_widget.hpp>

#include <unordered_map>
#include <thread>

namespace tesseract_rviz
{
class JointTrajectoryMonitorPropertiesPrivate
{
public:
  std::shared_ptr<rclcpp::Node> rviz_node;
  std::shared_ptr<rclcpp::Node> node;
  std::shared_ptr<std::thread> internal_node_spinner;
  rviz_common::Display* parent;
  rviz_common::properties::Property* main_property;
  tesseract_gui::JointTrajectoryWidget* widget;

  rviz_common::properties::BoolProperty* legacy_main;
  rviz_common::properties::StringProperty* legacy_joint_trajectory_topic_property;

  rviz_common::properties::BoolProperty* tesseract_main;
  rviz_common::properties::StringProperty* tesseract_joint_trajectory_topic_property;

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr legacy_joint_trajectory_sub;
  rclcpp::Subscription<tesseract_msgs::msg::Trajectory>::SharedPtr tesseract_joint_trajectory_sub;

  void legacyJointTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
  void tesseractJointTrajectoryCallback(const tesseract_msgs::msg::Trajectory::SharedPtr msg);
};

JointTrajectoryMonitorProperties::JointTrajectoryMonitorProperties(rviz_common::Display* parent,
                                                                   rviz_common::properties::Property* main_property)
  : data_(std::make_unique<JointTrajectoryMonitorPropertiesPrivate>())
{
  data_->parent = parent;

  data_->main_property = main_property;
  if (data_->main_property == nullptr)
    data_->main_property = data_->parent;

  data_->legacy_main = new rviz_common::properties::BoolProperty("Legacy Joint Trajectory",
                                                                 true,
                                                                 "This will monitor this topic for "
                                                                 "trajectory_msgs::JointTrajectory "
                                                                 "messages.",
                                                                 data_->main_property,
                                                                 SLOT(onLegacyJointTrajectoryChanged()),
                                                                 this);
  data_->legacy_main->setDisableChildrenIfFalse(true);

  data_->legacy_joint_trajectory_topic_property =
      new rviz_common::properties::StringProperty("topic",
                                                  "/joint_trajectory",
                                                  "This will monitor this topic for trajectory_msgs::JointTrajectory "
                                                  "messages.",
                                                  data_->legacy_main,
                                                  SLOT(onLegacyJointTrajectoryTopicChanged()),
                                                  this);

  data_->tesseract_main = new rviz_common::properties::BoolProperty("Tesseract Joint Trajectory",
                                                                    true,
                                                                    "This will monitor this topic for "
                                                                    "trajectory_msgs::JointTrajectory "
                                                                    "messages.",
                                                                    data_->main_property,
                                                                    SLOT(onTesseractJointTrajectoryChanged()),
                                                                    this);
  data_->tesseract_main->setDisableChildrenIfFalse(true);

  data_->tesseract_joint_trajectory_topic_property =
      new rviz_common::properties::StringProperty("Joint Trajectory Topic",
                                                  "/tesseract_trajectory",
                                                  "This will monitor this topic for tesseract_msgs::Trajectory "
                                                  "messages.",
                                                  data_->tesseract_main,
                                                  SLOT(onTesseractJointTrajectoryTopicChanged()),
                                                  this);
}

JointTrajectoryMonitorProperties::~JointTrajectoryMonitorProperties()
{
  if (data_->internal_node_spinner->joinable())
    data_->internal_node_spinner->join();
}

void JointTrajectoryMonitorProperties::onInitialize(tesseract_gui::JointTrajectoryWidget* widget,
                                                    std::shared_ptr<rclcpp::Node> rviz_node)
{
  data_->rviz_node = rviz_node;

  data_->node = std::make_shared<rclcpp::Node>(std::string(data_->rviz_node->get_name()) + "TesseractWorkbench_"
                                                                                           "JointTrajectory_Node");
  data_->internal_node_spinner = std::make_shared<std::thread>(std::thread{ [this]() {
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(data_->node);
    executor.spin();
  } });
  data_->internal_node_spinner->detach();
  data_->widget = widget;
  onLegacyJointTrajectoryTopicConnect();
  onTesseractJointTrajectoryTopicConnect();
}

void JointTrajectoryMonitorProperties::load(const rviz_common::Config& config)
{
  QString topic;
  if (config.mapGetString("tesseract::LegacyJointTrajectoryTopic", &topic))
    data_->legacy_joint_trajectory_topic_property->setString(topic);

  if (config.mapGetString("tesseract::TesseractJointTrajectoryTopic", &topic))
    data_->tesseract_joint_trajectory_topic_property->setString(topic);
}

void JointTrajectoryMonitorProperties::save(rviz_common::Config config) const
{
  config.mapSetValue("tesseract::LegacyJointTrajectoryTopic",
                     data_->legacy_joint_trajectory_topic_property->getString());
  config.mapSetValue("tesseract::TesseractJointTrajectoryTopic",
                     data_->tesseract_joint_trajectory_topic_property->getString());
}

void JointTrajectoryMonitorProperties::onLegacyJointTrajectoryTopicConnect()
{
  data_->legacy_joint_trajectory_sub = data_->node->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      data_->legacy_joint_trajectory_topic_property->getStdString(),
      20,
      std::bind(
          &JointTrajectoryMonitorPropertiesPrivate::legacyJointTrajectoryCallback, *data_, std::placeholders::_1));
}

void JointTrajectoryMonitorProperties::onTesseractJointTrajectoryTopicConnect()
{
  data_->tesseract_joint_trajectory_sub = data_->node->create_subscription<tesseract_msgs::msg::Trajectory>(
      data_->tesseract_joint_trajectory_topic_property->getStdString(),
      20,
      std::bind(
          &JointTrajectoryMonitorPropertiesPrivate::tesseractJointTrajectoryCallback, *data_, std::placeholders::_1));
}

void JointTrajectoryMonitorProperties::onLegacyJointTrajectoryTopicDisconnect()
{
  data_->legacy_joint_trajectory_sub.reset();
}

void JointTrajectoryMonitorProperties::onTesseractJointTrajectoryTopicDisconnect()
{
  data_->tesseract_joint_trajectory_sub.reset();
}

void JointTrajectoryMonitorProperties::onLegacyJointTrajectoryTopicChanged()
{
  onLegacyJointTrajectoryTopicDisconnect();
  onLegacyJointTrajectoryTopicConnect();
}

void JointTrajectoryMonitorProperties::onTesseractJointTrajectoryTopicChanged()
{
  onTesseractJointTrajectoryTopicDisconnect();
  onTesseractJointTrajectoryTopicConnect();
}

void JointTrajectoryMonitorProperties::onLegacyJointTrajectoryChanged()
{
  if (data_->legacy_main->getBool())
    onLegacyJointTrajectoryTopicConnect();
  else
    onLegacyJointTrajectoryTopicDisconnect();
}

void JointTrajectoryMonitorProperties::onTesseractJointTrajectoryChanged()
{
  if (data_->tesseract_main->getBool())
    onTesseractJointTrajectoryTopicConnect();
  else
    onTesseractJointTrajectoryTopicDisconnect();
}

void JointTrajectoryMonitorPropertiesPrivate::legacyJointTrajectoryCallback(
    const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  if (msg->joint_names.empty())
    return;

  if (msg->points.empty())
    return;

  std::unordered_map<std::string, double> initial_state;
  for (std::size_t i = 0; i < msg->joint_names.size(); ++i)
    initial_state[msg->joint_names[i]] = msg->points[0].positions[i];

  tesseract_common::JointTrajectorySet trajectory_set(initial_state);
  tesseract_common::JointTrajectory joint_trajectory = tesseract_rosutils::fromMsg(*msg);
  trajectory_set.appendJointTrajectory(joint_trajectory);
  widget->addJointTrajectorySet(trajectory_set);
}

void JointTrajectoryMonitorPropertiesPrivate::tesseractJointTrajectoryCallback(
    const tesseract_msgs::msg::Trajectory::SharedPtr msg)
{
  try
  {
    tesseract_common::JointTrajectorySet trajectory_set;

    // Get environment initial state
    std::unordered_map<std::string, double> initial_state;
    for (const auto& pair_msg : msg->initial_state)
      initial_state[pair_msg.first] = pair_msg.second;

    // Get environment information
    tesseract_environment::Environment::UPtr environment = tesseract_rosutils::fromMsg(msg->environment);
    tesseract_environment::Commands commands = tesseract_rosutils::fromMsg(msg->commands);

    if (environment != nullptr)
    {
      trajectory_set = tesseract_common::JointTrajectorySet(std::move(environment));
    }
    else if (!commands.empty())
    {
      trajectory_set = tesseract_common::JointTrajectorySet(initial_state, commands);
    }
    else
    {
      trajectory_set = tesseract_common::JointTrajectorySet(initial_state);
    }

    if (!msg->ns.empty())
      trajectory_set.setNamespace(msg->ns);

    for (const auto& joint_trajectory_msg : msg->joint_trajectories)
    {
      tesseract_common::JointTrajectory joint_trajectory = tesseract_rosutils::fromMsg(joint_trajectory_msg);
      trajectory_set.appendJointTrajectory(joint_trajectory);
    }
    widget->addJointTrajectorySet(trajectory_set);
  }
  catch (...)
  {
    parent->setStatus(
        rviz_common::properties::StatusProperty::Error, "Tesseract", "Failed to process trajectory message!");
  }
}

}  // namespace tesseract_rviz
