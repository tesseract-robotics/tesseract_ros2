#ifndef TESSERACT_RVIZ_WORKBENCH_DISPLAY_H
#define TESSERACT_RVIZ_WORKBENCH_DISPLAY_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <rviz_common/display.hpp>
#ifndef Q_MOC_RUN
#include <tesseract_environment/environment.h>
#endif
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_rviz
{
struct WorkbenchDisplayPrivate;

class WorkbenchDisplay : public rviz_common::Display
{
  Q_OBJECT
public:
  WorkbenchDisplay();
  ~WorkbenchDisplay() override;

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

  std::unique_ptr<WorkbenchDisplayPrivate> data_;
};
}  // namespace tesseract_rviz

#endif  // TESSERACT_RVIZ_WORKBENCH_DISPLAY_H
