#ifndef TESSERACT_RVIZ_ENVIRONMENT_MONITOR_PROPERTIES_H
#define TESSERACT_RVIZ_ENVIRONMENT_MONITOR_PROPERTIES_H

#include <memory>
#include <QObject>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <tesseract_qt/common/component_info.h>
#include <tesseract_msgs/msg/environment.hpp>

namespace rviz_common
{
class Display;
class Config;
namespace properties
{
class Property;
}  // namespace properties
}  // namespace rviz_common

namespace Ogre
{
class SceneManager;
class SceneNode;
}  // namespace Ogre

namespace tesseract_gui
{
class EnvironmentWidget;
}

namespace tesseract_rviz
{
class EnvironmentMonitorProperties : public QObject
{
  Q_OBJECT
public:
  EnvironmentMonitorProperties(rviz_common::Display* parent,
                               std::string monitor_namespace,
                               rviz_common::properties::Property* main_property = nullptr);
  ~EnvironmentMonitorProperties() override;

  void onInitialize(Ogre::SceneManager* scene_manager,
                    Ogre::SceneNode* scene_node,
                    rviz_common::DisplayContext* context);

  /**
   * @brief Return the component info based on the settings of the object
   * @return The component info
   */
  std::shared_ptr<const tesseract_gui::ComponentInfo> getComponentInfo() const;

  void resetMonitor();

Q_SIGNALS:
  void componentInfoChanged(std::shared_ptr<const tesseract_gui::ComponentInfo> component_info);

public Q_SLOTS:
  void onDisplayModeChanged();
  void onURDFDescriptionChanged();
  void onEnvironmentTopicChanged();
  void onEnvironmentSnapshotTopicChanged();
  void onJointStateTopicChanged();

protected:
  struct Implementation;
  std::unique_ptr<Implementation> data_;

  void snapshotCallback(const tesseract_msgs::msg::Environment::SharedPtr msg);
};

}  // namespace tesseract_rviz

#endif  // TESSERACT_RVIZ_ENVIRONMENT_MONITOR_PROPERTIES_H
