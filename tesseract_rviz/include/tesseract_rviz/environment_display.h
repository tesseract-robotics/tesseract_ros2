#ifndef TESSERACT_RVIZ_ENVIRONMENT_DISPLAY_H
#define TESSERACT_RVIZ_ENVIRONMENT_DISPLAY_H

#include <rviz_common/display.hpp>
#include <tesseract_qt/common/component_info.h>

namespace tesseract_rviz
{
class EnvironmentDisplay : public rviz_common::Display
{
  Q_OBJECT
public:
  EnvironmentDisplay();
  ~EnvironmentDisplay() override;

  void update(float wall_dt, float ros_dt) override;
  void reset() override;

public Q_SLOTS:
  void onEnableChanged() override;
  void onComponentInfoChanged(std::shared_ptr<const tesseract_gui::ComponentInfo> component_info);

protected:
  // overrides from Display
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;

private:
  struct Implementation;
  std::unique_ptr<Implementation> data_;
};
}  // namespace tesseract_rviz
#endif  // TESSERACT_RVIZ_ENVIRONMENT_DISPLAY_H
