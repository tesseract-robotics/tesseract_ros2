#ifndef TESSERACT_RVIZ_ENVIRONMENT_MONITOR_PROPERTIES_H
#define TESSERACT_RVIZ_ENVIRONMENT_MONITOR_PROPERTIES_H

#include <memory>
#include <QObject>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>

namespace rviz_common
{
class Display;
class Config;
namespace properties
{
class Property;
}  // namespace properties
}  // namespace rviz_common

namespace tesseract_gui
{
class EnvironmentWidgetConfig;
}

namespace tesseract_rviz
{
class ROSEnvironmentWidget;
struct EnvironmentMonitorPropertiesPrivate;

class EnvironmentMonitorProperties : public QObject
{
  Q_OBJECT
public:
  EnvironmentMonitorProperties(rviz_common::Display* parent,
                               std::string monitor_namespace,
                               rviz_common::properties::Property* main_property = nullptr);
  ~EnvironmentMonitorProperties() override;

  void onInitialize(ROSEnvironmentWidget* widget, rviz_common::DisplayContext* context);

  /**
   * @brief Return the config based on the settings of the object
   * @return The environment config
   */
  std::shared_ptr<tesseract_gui::EnvironmentWidgetConfig> getConfig() const;

  void load(const rviz_common::Config& config);
  void save(rviz_common::Config config) const;

public Q_SLOTS:
  void onDisplayModeChanged();
  void onURDFDescriptionChanged();
  void onEnvironmentTopicChanged();
  void onJointStateTopicChanged();

protected:
  std::unique_ptr<EnvironmentMonitorPropertiesPrivate> data_;
};

}  // namespace tesseract_rviz

#endif  // TESSERACT_RVIZ_ENVIRONMENT_MONITOR_PROPERTIES_H
