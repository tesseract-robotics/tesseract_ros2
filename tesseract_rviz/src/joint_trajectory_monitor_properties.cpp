#include <tesseract_rviz/joint_trajectory_monitor_properties.h>
#include <tesseract_rviz/conversions.h>

#include <tesseract_qt/joint_trajectory/widgets/joint_trajectory_widget.h>

#include <tesseract_qt/common/events/joint_trajectory_events.h>
#include <tesseract_qt/common/joint_trajectory_set.h>
#include <tesseract_qt/common/component_info.h>
#include <tesseract_qt/common/component_info_manager.h>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <tesseract_msgs/msg/trajectory.hpp>
#include <tesseract_rosutils/utils.h>

#include <rviz_common/display.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/panel_dock_widget.hpp>

#include <unordered_map>
#include <thread>

#include <QApplication>

namespace tesseract_rviz
{
struct JointTrajectoryMonitorProperties::Implementation
{
  // public:
  std::shared_ptr<rclcpp::Node> rviz_node;
  std::shared_ptr<rclcpp::Node> node;
  rclcpp::executors::MultiThreadedExecutor::SharedPtr internal_node_executor;
  std::shared_ptr<std::thread> internal_node_spinner;
  rviz_common::Display* parent;
  rviz_common::properties::Property* main_property;

  std::shared_ptr<const tesseract_gui::ComponentInfo> component_info;

  rviz_common::properties::BoolProperty* legacy_main;
  rviz_common::properties::RosTopicProperty* legacy_joint_trajectory_topic_property;

  rviz_common::properties::BoolProperty* tesseract_main;
  rviz_common::properties::RosTopicProperty* tesseract_joint_trajectory_topic_property;

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr legacy_joint_trajectory_sub;
  rclcpp::Subscription<tesseract_msgs::msg::Trajectory>::SharedPtr tesseract_joint_trajectory_sub;

  void legacyJointTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::ConstSharedPtr msg)
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
    tesseract_gui::events::JointTrajectoryAdd event(component_info, trajectory_set);
    QApplication::sendEvent(qApp, &event);
  }

  void tesseractJointTrajectoryCallback(const tesseract_msgs::msg::Trajectory::ConstSharedPtr msg)
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
        tesseract_gui::events::JointTrajectoryAdd event(component_info, trajectory_set);
        QApplication::sendEvent(qApp, &event);
      }
    }
    catch (...)
    {
      parent->setStatus(
          rviz_common::properties::StatusProperty::Error, "Tesseract", "Failed to process trajectory message!");
    }
  }
};

JointTrajectoryMonitorProperties::JointTrajectoryMonitorProperties(rviz_common::Display* parent,
                                                                   rviz_common::properties::Property* main_property)
  : data_(std::make_unique<Implementation>())
{
  data_->parent = parent;
  data_->component_info = tesseract_gui::ComponentInfoManager::create("rviz_scene");

  data_->main_property = main_property;
  if (data_->main_property == nullptr)
    data_->main_property = data_->parent;

  data_->legacy_main = new rviz_common::properties::BoolProperty("Legacy Joint Trajectory",
                                                                 true,
                                                                 "This will monitor this topic for "
                                                                 "trajectory_msgs::msg::JointTrajectory "
                                                                 "messages.",
                                                                 data_->main_property,
                                                                 SLOT(onLegacyJointTrajectoryChanged()),
                                                                 this);
  data_->legacy_main->setDisableChildrenIfFalse(true);

  data_->legacy_joint_trajectory_topic_property = new rviz_common::properties::RosTopicProperty(
      "topic",
      "/joint_trajectory",
      rosidl_generator_traits::data_type<trajectory_msgs::msg::JointTrajectory>(),
      "This will monitor this topic for trajectory_msgs::msg::JointTrajectory messages.",
      data_->legacy_main,
      SLOT(onLegacyJointTrajectoryTopicChanged()),
      this);

  data_->tesseract_main = new rviz_common::properties::BoolProperty("Tesseract Joint Trajectory",
                                                                    true,
                                                                    "This will monitor this topic for "
                                                                    "trajectory_msgs::msg::JointTrajectory "
                                                                    "messages.",
                                                                    data_->main_property,
                                                                    SLOT(onTesseractJointTrajectoryChanged()),
                                                                    this);
  data_->tesseract_main->setDisableChildrenIfFalse(true);

  data_->tesseract_joint_trajectory_topic_property = new rviz_common::properties::RosTopicProperty(
      "Joint Trajectory Topic",
      "/tesseract_trajectory",
      rosidl_generator_traits::data_type<tesseract_msgs::msg::Trajectory>(),
      "This will monitor this topic for tesseract_msgs::msg::Trajectory "
      "messages.",
      data_->tesseract_main,
      SLOT(onTesseractJointTrajectoryTopicChanged()),
      this);
}

JointTrajectoryMonitorProperties::~JointTrajectoryMonitorProperties()
{
  data_->internal_node_executor->cancel();
  if (data_->internal_node_spinner->joinable())
    data_->internal_node_spinner->join();
}

void JointTrajectoryMonitorProperties::onInitialize(rviz_common::DisplayContext* context)
{
  auto ros_node_abstraction = context->getRosNodeAbstraction().lock();
  data_->rviz_node = ros_node_abstraction->get_raw_node();
  data_->node = std::make_shared<rclcpp::Node>(data_->parent->getNameStd() + "_JointTrajectory_Node",
                                               data_->rviz_node->get_fully_qualified_name());

  data_->legacy_joint_trajectory_topic_property->initialize(ros_node_abstraction);
  data_->tesseract_joint_trajectory_topic_property->initialize(ros_node_abstraction);

  data_->internal_node_executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  data_->internal_node_spinner = std::make_shared<std::thread>(std::thread{ [this]() {
    data_->internal_node_executor->add_node(data_->node);
    data_->internal_node_executor->spin();
    data_->internal_node_executor->remove_node(data_->node);
  } });

  onLegacyJointTrajectoryTopicConnect();
  onTesseractJointTrajectoryTopicConnect();
}

void JointTrajectoryMonitorProperties::setComponentInfo(
    std::shared_ptr<const tesseract_gui::ComponentInfo> component_info)
{
  data_->component_info = std::move(component_info);
}

std::shared_ptr<const tesseract_gui::ComponentInfo> JointTrajectoryMonitorProperties::getComponentInfo() const
{
  return data_->component_info;
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
      std::bind(&JointTrajectoryMonitorProperties::Implementation::legacyJointTrajectoryCallback,
                *data_,
                std::placeholders::_1));
}

void JointTrajectoryMonitorProperties::onTesseractJointTrajectoryTopicConnect()
{
  data_->tesseract_joint_trajectory_sub = data_->node->create_subscription<tesseract_msgs::msg::Trajectory>(
      data_->tesseract_joint_trajectory_topic_property->getStdString(),
      20,
      std::bind(&JointTrajectoryMonitorProperties::Implementation::tesseractJointTrajectoryCallback,
                *data_,
                std::placeholders::_1));
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
}  // namespace tesseract_rviz
