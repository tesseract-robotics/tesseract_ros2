#ifndef TESSERACT_RVIZ_ENVIRONMENT_DISPLAY_H
#define TESSERACT_RVIZ_ENVIRONMENT_DISPLAY_H

#include <rviz_common/display.hpp>
#include <tesseract_environment/environment.h>
#include <tesseract_qt/common/entity_container.h>

namespace tesseract_rviz
{
struct EnvironmentDisplayPrivate;

class EnvironmentDisplay : public rviz_common::Display
{
  Q_OBJECT
public:
  EnvironmentDisplay();
  ~EnvironmentDisplay() override;

  void update(float wall_dt, float ros_dt) override;
  void reset() override;

  void load(const rviz_common::Config& config) override;
  void save(rviz_common::Config config) const override;

public Q_SLOTS:
  void onEnableChanged() override;

protected:
  // overrides from Display
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;

  std::unique_ptr<EnvironmentDisplayPrivate> data_;
};
}  // namespace tesseract_rviz
#endif  // TESSERACT_RVIZ_ENVIRONMENT_DISPLAY_H
