#ifndef TESSERACT_RVIZ_JOINT_TRAJECTORY_PROPERTIES_H
#define TESSERACT_RVIZ_JOINT_TRAJECTORY_PROPERTIES_H

#include <memory>
#include <QObject>


namespace rviz_common
{
class Display;
class Config;
namespace properties
{
class Property;
} // namespace properties
}  // namespace rviz_common

namespace tesseract_gui
{
class JointTrajectoryWidget;
class EnvironmentWidgetConfig;
}  // namespace tesseract_gui

namespace tesseract_rviz
{
class JointTrajectoryMonitorPropertiesPrivate;

class JointTrajectoryMonitorProperties : public QObject
{
  Q_OBJECT
public:
  /**
   * @brief JointTrajectoryMonitorProperties
   * @param parent The parent rviz display
   * @param main_property The property to add these properties to. If nullptr then they are added to the display
   */
  JointTrajectoryMonitorProperties(rviz_common::Display* parent, rviz_common::properties::Property* main_property = nullptr);
  ~JointTrajectoryMonitorProperties() override;

  void onInitialize(tesseract_gui::JointTrajectoryWidget* widget);

  void load(const rviz_common::Config& config);
  void save(rviz_common::Config config) const;

public Q_SLOTS:
  void onLegacyJointTrajectoryTopicConnect();
  void onTesseractJointTrajectoryTopicConnect();
  void onLegacyJointTrajectoryTopicDisconnect();
  void onTesseractJointTrajectoryTopicDisconnect();

private Q_SLOTS:
  void onLegacyJointTrajectoryTopicChanged();
  void onTesseractJointTrajectoryTopicChanged();
  void onLegacyJointTrajectoryChanged();
  void onTesseractJointTrajectoryChanged();

protected:
  std::unique_ptr<JointTrajectoryMonitorPropertiesPrivate> data_;
};

}  // namespace tesseract_rviz

#endif  // TESSERACT_RVIZ_JOINT_TRAJECTORY_PROPERTIES_H
