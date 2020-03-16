#ifndef TESSERACT_RVIZ_ENVIRONMENT_MONITORING_H
#define TESSERACT_RVIZ_ENVIRONMENT_MONITORING_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <rviz_common/display.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#ifndef Q_MOC_RUN

TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <rclcpp/rclcpp.hpp>
#include <tesseract_msgs/srv/modify_environment.hpp>
#include <tesseract_msgs/srv/get_environment_changes.hpp>
#include <tesseract_msgs/msg/tesseract_state.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tesseract/tesseract.h>
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
  using SharedPtr = std::shared_ptr<EnvironmentWidget>;
  using ConstSharedPtr = std::shared_ptr<const EnvironmentWidget>;

  EnvironmentWidget(rviz_common::properties::Property* widget, rviz_common::Display* display, const std::string& widget_ns = std::string());

  virtual ~EnvironmentWidget();

  void onInitialize(VisualizationWidget::SharedPtr visualization,
                    tesseract::Tesseract::Ptr tesseract,
                    rviz_common::DisplayContext* context,
                    rclcpp::Node::SharedPtr update_nh,
                    bool update_state,
                    QString tesseract_state_topic = "");

  void onEnable();
  void onDisable();
  void onUpdate();
  void onReset();

  /** @brief Returns the ID of this EnvironmentWidget instance which is associated with the default namespace */
  int getId() const { return environment_widget_id_; }

private Q_SLOTS:
  void changedURDFDescription();
  void changedEnvironmentNamespace();
  void changedRootLinkName();
  void changedTesseractStateTopic();
  void changedURDFSceneAlpha();
  void changedEnableLinkHighlight();
  void changedEnableVisualVisible();
  void changedEnableCollisionVisible();
  void changedAllLinks();

protected:
  rviz_common::properties::Property* widget_;
  rviz_common::Display* display_;
  VisualizationWidget::SharedPtr visualization_;
  tesseract::Tesseract::Ptr tesseract_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<tesseract_msgs::msg::TesseractState>::SharedPtr tesseract_state_subscriber_; /**< @brief subscriber for getting environment updates */
  rclcpp::Service<tesseract_msgs::srv::ModifyEnvironment>::SharedPtr modify_environment_server_;      /**< @brief host a service for modifying the environment */
  rclcpp::Service<tesseract_msgs::srv::GetEnvironmentChanges>::SharedPtr get_environment_changes_server_; /**< @brief host a service for getting the environment changes */
  bool update_required_;
  bool update_state_;    /**< @brief Update visualization current state from environment message */
  bool load_tesseract_;  // for delayed initialization
  std::map<std::string, std_msgs::msg::ColorRGBA> highlights_;

  void loadEnvironment();

  /** @brief Set the scene node's position, given the target frame and the planning frame */
  //  void calculateOffsetPosition();

  //  void setLinkColor(const tesseract_msgs::TesseractState::_object_colors_type& link_colors);
  //  void setLinkColor(Robot* robot, const std::string& link_name, const QColor& color);
  //  void unsetLinkColor(Robot* robot, const std::string& link_name);

  /** @brief Callback for new tesseract state message */
  void newTesseractStateCallback(const tesseract_msgs::msg::TesseractState::ConstSharedPtr state);

  /** @brief Callback for modifying the environment via service request */
  bool modifyEnvironmentCallback(const std::shared_ptr<rmw_request_id_t> request_header,
      tesseract_msgs::srv::ModifyEnvironment::Request::SharedPtr req,
                                 tesseract_msgs::srv::ModifyEnvironment::Response::SharedPtr res);

  /** @brief Callback for get the environment changes via service request */
  bool getEnvironmentChangesCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                     tesseract_msgs::srv::GetEnvironmentChanges::Request::SharedPtr req,
                                     tesseract_msgs::srv::GetEnvironmentChanges::Response::SharedPtr res);

  /** @brief Apply a list of commands to the environment. This used by both services and topics for updating environment
   * visualization */
  bool applyEnvironmentCommands(const std::vector<tesseract_msgs::msg::EnvironmentCommand>& commands);

  rviz_common::properties::Property* main_property_;
  rviz_common::properties::StringProperty* urdf_description_property_;
  rviz_common::properties::StringProperty* environment_namespace_property_;
  rviz_common::properties::RosTopicProperty* tesseract_state_topic_property_;
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
