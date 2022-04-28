#ifndef TESSERACT_RVIZ_ENVIRONMENT_MONITORING_H
#define TESSERACT_RVIZ_ENVIRONMENT_MONITORING_H

#include <tesseract_common/macros.h>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display.hpp>
#include <tesseract_monitoring/environment_monitor.h>
#include <std_msgs/msg/color_rgba.hpp>

#ifndef Q_MOC_RUN

TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_environment/environment.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#endif

#include <tesseract_rviz/render_tools/visualization_widget.h>

namespace rviz_common::properties
{
class Property;
class RosTopicProperty;
class StringProperty;
class FloatProperty;
}  // namespace rviz

namespace tesseract_rviz
{
class EnvironmentWidget : public QObject
{
  Q_OBJECT

public:
  using Ptr = std::shared_ptr<EnvironmentWidget>;
  using ConstPtr = std::shared_ptr<const EnvironmentWidget>;

  EnvironmentWidget(rviz_common::properties::Property* widget, rviz_common::Display* display, const std::string& widget_ns = std::string());

  virtual ~EnvironmentWidget();

  void onInitialize(VisualizationWidget::Ptr visualization,
                    tesseract_environment::Environment::Ptr env,
                    rviz_common::DisplayContext* context,
                    rclcpp::Node::SharedPtr update_node,
                    bool update_state);

  void onEnable();
  void onDisable();
  void onUpdate();
  void onReset();

  /** @brief Returns the ID of this EnvironmentWidget instance which is associated with the default namespace */
  int getId() const { return environment_widget_id_; }

private Q_SLOTS:
  void changedURDFDescription();
  void changedRootLinkName();
  void changedDisplayMode();
  void changedDisplayModeString();
  void changedURDFSceneAlpha();
  void changedEnableLinkHighlight();
  void changedEnableVisualVisible();
  void changedEnableCollisionVisible();
  void changedAllLinks();

protected:
  rviz_common::properties::Property* widget_;
  rviz_common::Display* display_;
  VisualizationWidget::Ptr visualization_;
  tesseract_environment::Environment::Ptr env_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::SubscriptionBase::SharedPtr robot_description_sub_;
  std::unique_ptr<tesseract_monitoring::EnvironmentMonitor> monitor_;
  int revision_{ 0 }; /**< The current revision of the visualization environment */
  bool update_required_;
  bool update_state_;    /**< @brief Update visualization current state from environment message */
  bool load_tesseract_;  // for delayed initialization
  std::map<std::string, std_msgs::msg::ColorRGBA> highlights_;
  std::chrono::high_resolution_clock::duration state_timestamp_{
    std::chrono::high_resolution_clock::now().time_since_epoch()
  };

  void loadEnvironment();

  /** @brief Set the scene node's position, given the target frame and the planning frame */
  //  void calculateOffsetPosition();

  //  void setLinkColor(const tesseract_msgs::TesseractState::_object_colors_type& link_colors);
  //  void setLinkColor(Robot* robot, const std::string& link_name, const QColor& color);
  //  void unsetLinkColor(Robot* robot, const std::string& link_name);

  /** @brief Callback for modifying the environment via service request */
  //void modifyEnvironmentCallback(
  //  tesseract_msgs::srv::ModifyEnvironment::Request::SharedPtr req,
  //  tesseract_msgs::srv::ModifyEnvironment::Response::SharedPtr res);

  /** @brief Callback for get the environment changes via service request */
  //void getEnvironmentChangesCallback(
  //  tesseract_msgs::srv::GetEnvironmentChanges::Request::SharedPtr req,
  //  tesseract_msgs::srv::GetEnvironmentChanges::Response::SharedPtr res);

  /** @brief Apply a list of commands to the environment. This used by both services and topics for updating environment
   * visualization */
  bool applyEnvironmentCommands(const tesseract_environment::Command& command);

  rviz_common::properties::Property* main_property_;
  rviz_common::properties::EnumProperty* display_mode_property_;
  rviz_common::properties::StringProperty* display_mode_string_property_;
  rviz_common::properties::StringProperty* urdf_description_property_;
  rviz_common::properties::StringProperty* environment_namespace_property_;
  rviz_common::properties::StringProperty* root_link_name_property_;
  rviz_common::properties::FloatProperty* alpha_property_;
  rviz_common::properties::BoolProperty* enable_link_highlight_;
  rviz_common::properties::BoolProperty* enable_visual_visible_;
  rviz_common::properties::BoolProperty* enable_collision_visible_;
  rviz_common::properties::BoolProperty* show_all_links_;

private:
  /** @brief Keeps track of how many EnvironmentWidgets have been created for the default namespace */
  static int environment_widget_counter_;
  /** @brief Keeps track of which EnvironmentWidget this is */
  int environment_widget_id_;

  std::string widget_ns_;
};
}  // namespace tesseract_rviz
#endif  // TESSERACT_RVIZ_ENVIRONMENT_MONITORING_H
