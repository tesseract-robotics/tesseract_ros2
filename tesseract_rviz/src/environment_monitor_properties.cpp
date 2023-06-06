#include <tesseract_rviz/environment_monitor_properties.h>
#include <tesseract_rviz/ros_scene_graph_render_manager.h>
#include <tesseract_rviz/ros_contact_results_render_manager.h>
#include <tesseract_rviz/conversions.h>

#include <tesseract_qt/common/component_info.h>
#include <tesseract_qt/common/environment_manager.h>
#include <tesseract_qt/common/environment_wrapper.h>

#include <tesseract_monitoring/environment_monitor.h>

#include <rclcpp/logging.hpp>
#include <tesseract_msgs/msg/environment_state.hpp>
#include <tesseract_msgs/msg/environment.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <rviz_common/display.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/panel_dock_widget.hpp>

#include <unordered_map>

namespace tesseract_rviz
{
struct EnvironmentMonitorProperties::Implementation
{
  std::shared_ptr<rclcpp::Node> rviz_node;
  std::shared_ptr<rclcpp::Node> node;
  rviz_common::Display* parent{ nullptr };
  rviz_common::properties::Property* main_property{ nullptr };
  Ogre::SceneManager* scene_manager{ nullptr };
  Ogre::SceneNode* scene_node{ nullptr };
  rclcpp::executors::MultiThreadedExecutor::SharedPtr internal_node_executor;
  std::shared_ptr<std::thread> internal_node_spinner;
  std::string monitor_namespace;
  tesseract_environment::EnvironmentMonitor::Ptr monitor;
  rclcpp::Subscription<tesseract_msgs::msg::Environment>::SharedPtr snapshot;

  tesseract_gui::ComponentInfo component_info{ "rviz_scene" };
  tesseract_gui::SceneGraphRenderManager::Ptr render_manager;
  tesseract_gui::ContactResultsRenderManager::Ptr contact_results_render_manager;

  rviz_common::properties::EnumProperty* display_mode_property{ nullptr };
  rviz_common::properties::StringProperty* urdf_description_string_property{ nullptr };
  rviz_common::properties::RosTopicProperty* environment_topic_property{ nullptr };
  rviz_common::properties::RosTopicProperty* environment_snapshot_topic_property{ nullptr };
  rviz_common::properties::RosTopicProperty* joint_state_topic_property{ nullptr };
};

EnvironmentMonitorProperties::EnvironmentMonitorProperties(rviz_common::Display* parent,
                                                           std::string monitor_namespace,
                                                           rviz_common::properties::Property* main_property)
  : data_(std::make_unique<Implementation>())
{
  data_->parent = parent;
  data_->monitor_namespace = monitor_namespace;

  data_->main_property = main_property;
  if (data_->main_property == nullptr)
    data_->main_property = data_->parent;

  data_->display_mode_property = new rviz_common::properties::EnumProperty("Display Mode",
                                                                           "URDF",
                                                                           "Leverage URDF or connect to monitor "
                                                                           "namespace",
                                                                           data_->main_property,
                                                                           SLOT(onDisplayModeChanged()),
                                                                           this);

  data_->display_mode_property->addOptionStd("URDF", 0);
  data_->display_mode_property->addOptionStd("Monitor", 1);
  data_->display_mode_property->addOptionStd("Snapshot", 2);

  data_->urdf_description_string_property =
      new rviz_common::properties::StringProperty("URDF Parameter",
                                                  "robot_description",
                                                  "The URDF parameter to use for creating the "
                                                  "environment within RViz",
                                                  data_->main_property,
                                                  SLOT(onURDFDescriptionChanged()),
                                                  this);

  data_->environment_topic_property = new rviz_common::properties::RosTopicProperty(
      "Monitor Topic",
      "/tesseract_environment",
      rosidl_generator_traits::data_type<tesseract_msgs::msg::EnvironmentState>(),
      "This will monitor this topic for environment changes.",
      data_->main_property,
      SLOT(onEnvironmentTopicChanged()),
      this);

  data_->environment_snapshot_topic_property = new rviz_common::properties::RosTopicProperty(
      "Snapshot Topic",
      "/tesseract_environment_snapshot",
      rosidl_generator_traits::data_type<tesseract_msgs::msg::Environment>(),
      "This will monitor this topic for environment snapshots.",
      data_->main_property,
      SLOT(onEnvironmentSnapshotTopicChanged()),
      this);

  data_->joint_state_topic_property =
      new rviz_common::properties::RosTopicProperty("Joint State Topic",
                                                    tesseract_monitoring::DEFAULT_JOINT_STATES_TOPIC.c_str(),
                                                    rosidl_generator_traits::data_type<sensor_msgs::msg::JointState>(),
                                                    "This will monitor this topic for joint state changes.",
                                                    data_->main_property,
                                                    SLOT(onJointStateTopicChanged()),
                                                    this);
}

EnvironmentMonitorProperties::~EnvironmentMonitorProperties()
{
  tesseract_gui::EnvironmentManager::remove(data_->component_info);
  data_->internal_node_executor->cancel();
  if (data_->internal_node_spinner->joinable())
    data_->internal_node_spinner->join();
}

void EnvironmentMonitorProperties::onInitialize(Ogre::SceneManager* scene_manager,
                                                Ogre::SceneNode* scene_node,
                                                rviz_common::DisplayContext* context)
{
  auto ros_node_abstraction = context->getRosNodeAbstraction().lock();
  data_->rviz_node = ros_node_abstraction->get_raw_node();
  data_->node = std::make_shared<rclcpp::Node>(data_->parent->getNameStd() + "_EnvMonitor_Node",
                                               data_->rviz_node->get_fully_qualified_name());
  data_->environment_topic_property->initialize(ros_node_abstraction);
  data_->joint_state_topic_property->initialize(ros_node_abstraction);

  data_->internal_node_executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  data_->internal_node_spinner = std::make_shared<std::thread>(std::thread{ [this]() {
    data_->internal_node_executor->add_node(data_->node);
    data_->internal_node_executor->spin();
  } });

  data_->scene_manager = scene_manager;
  data_->scene_node = scene_node;
  onDisplayModeChanged();
}

tesseract_gui::ComponentInfo EnvironmentMonitorProperties::getComponentInfo() const { return data_->component_info; }

void EnvironmentMonitorProperties::load(const rviz_common::Config& config)
{
  QString mode;
  if (config.mapGetString("tesseract::EnvMonitorMode", &mode))
    data_->display_mode_property->setString(mode);

  QString urdf_description;
  if (config.mapGetString("tesseract::EnvMonitorURDFDescription", &urdf_description))
    data_->urdf_description_string_property->setString(urdf_description);

  QString topic;
  if (config.mapGetString("tesseract::EnvMonitorTopic", &topic))
    data_->environment_topic_property->setString(topic);

  if (config.mapGetString("tesseract::EnvMonitorSnapshotTopic", &topic))
    data_->environment_snapshot_topic_property->setString(topic);

  if (config.mapGetString("tesseract::EnvMonitorJointStateTopic", &topic))
    data_->joint_state_topic_property->setString(topic);
}

void EnvironmentMonitorProperties::save(rviz_common::Config config) const
{
  config.mapSetValue("tesseract::EnvMonitorMode", data_->display_mode_property->getString());
  config.mapSetValue("tesseract::EnvMonitorURDFDescription", data_->urdf_description_string_property->getString());
  config.mapSetValue("tesseract::EnvMonitorTopic", data_->environment_topic_property->getString());
  config.mapSetValue("tesseract::EnvMonitorSnapshotTopic", data_->environment_topic_property->getString());
  config.mapSetValue("tesseract::EnvMonitorJointStateTopic", data_->joint_state_topic_property->getString());
}

void EnvironmentMonitorProperties::onDisplayModeChanged()
{
  if (data_->display_mode_property->getOptionInt() == 0)
  {
    data_->environment_topic_property->setHidden(true);
    data_->environment_snapshot_topic_property->setHidden(true);
    data_->snapshot.reset();

    data_->urdf_description_string_property->setHidden(false);
    onURDFDescriptionChanged();
  }
  else if (data_->display_mode_property->getOptionInt() == 1)
  {
    data_->urdf_description_string_property->setHidden(true);
    data_->environment_snapshot_topic_property->setHidden(true);
    data_->snapshot.reset();

    data_->environment_topic_property->setHidden(false);
    onEnvironmentTopicChanged();
  }
  else if (data_->display_mode_property->getOptionInt() == 2)
  {
    data_->urdf_description_string_property->setHidden(true);
    data_->environment_topic_property->setHidden(true);
    data_->environment_snapshot_topic_property->setHidden(false);
    onEnvironmentSnapshotTopicChanged();
  }
  onJointStateTopicChanged();
}

void EnvironmentMonitorProperties::onURDFDescriptionChanged()
{
  if (data_->scene_manager == nullptr || data_->scene_node == nullptr)
    return;

  data_->parent->deleteStatus("Tesseract");

  tesseract_gui::EnvironmentManager::remove(data_->component_info);
  data_->render_manager = nullptr;
  data_->contact_results_render_manager = nullptr;

  std::string urdf_xml_string, srdf_xml_string;
  if (!data_->rviz_node->get_parameter(data_->urdf_description_string_property->getStdString(), urdf_xml_string))
    urdf_xml_string = data_->rviz_node->declare_parameter(data_->urdf_description_string_property->getStdString(), "");
  if (!data_->rviz_node->get_parameter(data_->urdf_description_string_property->getStdString() + "_semantic",
                                       srdf_xml_string))
    srdf_xml_string =
        data_->rviz_node->declare_parameter(data_->urdf_description_string_property->getStdString() + "_semantic", "");

  auto env = std::make_shared<tesseract_environment::Environment>();
  auto locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  if (env->init(urdf_xml_string, srdf_xml_string, locator))
  {
    if (data_->monitor != nullptr)
      data_->monitor->shutdown();

    data_->monitor = std::make_unique<tesseract_monitoring::ROSEnvironmentMonitor>(
        data_->node, env, data_->urdf_description_string_property->getStdString());
    if (data_->monitor != nullptr)
    {
      data_->render_manager =
          std::make_shared<ROSSceneGraphRenderManager>(data_->component_info, data_->scene_manager, data_->scene_node);
      data_->contact_results_render_manager = std::make_shared<ROSContactResultsRenderManager>(
          data_->component_info, data_->scene_manager, data_->scene_node);

      Q_EMIT componentInfoChanged(data_->component_info);

      auto env_wrapper =
          std::make_shared<tesseract_gui::MonitorEnvironmentWrapper>(data_->component_info, data_->monitor);
      tesseract_gui::EnvironmentManager::set(env_wrapper);

      onJointStateTopicChanged();
    }
  }
  else
  {
    data_->parent->setStatus(rviz_common::properties::StatusProperty::Error, "Tesseract", "URDF file failed to parse");
  }
}

void EnvironmentMonitorProperties::onEnvironmentTopicChanged()
{
  if (data_->scene_manager == nullptr || data_->scene_node == nullptr)
    return;

  data_->parent->deleteStatus("Tesseract");

  if (data_->monitor != nullptr)
    data_->monitor->shutdown();

  tesseract_gui::EnvironmentManager::remove(data_->component_info);
  data_->render_manager = nullptr;
  data_->contact_results_render_manager = nullptr;

  auto env = std::make_shared<tesseract_environment::Environment>();
  data_->monitor =
      std::make_unique<tesseract_monitoring::ROSEnvironmentMonitor>(data_->node, env, data_->monitor_namespace);

  if (data_->monitor != nullptr)
  {
    std::string ns = getEnvNamespaceFromTopic(data_->environment_topic_property->getStdString());
    if (!ns.empty())
      data_->monitor->startMonitoringEnvironment(ns);
    else
      data_->parent->setStatus(
          rviz_common::properties::StatusProperty::Error, "Tesseract", "Invalid environment monitor topic!");

    data_->render_manager =
        std::make_shared<ROSSceneGraphRenderManager>(data_->component_info, data_->scene_manager, data_->scene_node);
    data_->contact_results_render_manager = std::make_shared<ROSContactResultsRenderManager>(
        data_->component_info, data_->scene_manager, data_->scene_node);

    Q_EMIT componentInfoChanged(data_->component_info);

    auto env_wrapper =
        std::make_shared<tesseract_gui::MonitorEnvironmentWrapper>(data_->component_info, data_->monitor);
    tesseract_gui::EnvironmentManager::set(env_wrapper);

    onJointStateTopicChanged();
  }
}

void EnvironmentMonitorProperties::snapshotCallback(const tesseract_msgs::msg::Environment::SharedPtr msg)
{
  if (data_->scene_manager == nullptr || data_->scene_node == nullptr)
    return;

  if (data_->monitor != nullptr)
    data_->monitor->shutdown();

  data_->parent->deleteStatus("Tesseract");

  tesseract_gui::EnvironmentManager::remove(data_->component_info);
  data_->render_manager = nullptr;
  data_->contact_results_render_manager = nullptr;

  tesseract_environment::Commands commands = tesseract_rosutils::fromMsg(msg->command_history);
  std::unordered_map<std::string, double> jv;
  tesseract_rosutils::fromMsg(jv, msg->joint_states);
  auto env = std::make_shared<tesseract_environment::Environment>();
  auto locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  env->setResourceLocator(locator);
  if (env->init(commands))
  {
    env->setState(jv);

    if (data_->monitor != nullptr)
      data_->monitor->shutdown();

    data_->monitor = std::make_unique<tesseract_monitoring::ROSEnvironmentMonitor>(
        data_->node, env, data_->urdf_description_string_property->getStdString());
    if (data_->monitor != nullptr)
    {
      data_->render_manager =
          std::make_shared<ROSSceneGraphRenderManager>(data_->component_info, data_->scene_manager, data_->scene_node);
      data_->contact_results_render_manager = std::make_shared<ROSContactResultsRenderManager>(
          data_->component_info, data_->scene_manager, data_->scene_node);

      Q_EMIT componentInfoChanged(data_->component_info);

      auto env_wrapper =
          std::make_shared<tesseract_gui::MonitorEnvironmentWrapper>(data_->component_info, data_->monitor);
      tesseract_gui::EnvironmentManager::set(env_wrapper);

      onJointStateTopicChanged();
    }
  }
  else
  {
    data_->parent->setStatus(
        rviz_common::properties::StatusProperty::Error, "Tesseract", "Snapshot failed to load from message");
  }
}

void EnvironmentMonitorProperties::onEnvironmentSnapshotTopicChanged()
{
  if (data_->scene_manager == nullptr || data_->scene_node == nullptr)
    return;

  if (data_->monitor != nullptr)
    data_->monitor->shutdown();

  // Shutdown the callback
  data_->snapshot.reset();

  tesseract_gui::EnvironmentManager::remove(data_->component_info);
  data_->render_manager = nullptr;
  data_->contact_results_render_manager = nullptr;

  // Connect to new topic
  data_->snapshot = data_->node->create_subscription<tesseract_msgs::msg::Environment>(
      data_->environment_snapshot_topic_property->getTopicStd(),
      10,
      std::bind(&EnvironmentMonitorProperties::snapshotCallback, this, std::placeholders::_1));
}

void EnvironmentMonitorProperties::onJointStateTopicChanged()
{
  if (data_->scene_manager == nullptr || data_->scene_node == nullptr)
    return;

  if (data_->monitor != nullptr)
    data_->monitor->startStateMonitor(data_->joint_state_topic_property->getTopicStd(), false);
}

}  // namespace tesseract_rviz
