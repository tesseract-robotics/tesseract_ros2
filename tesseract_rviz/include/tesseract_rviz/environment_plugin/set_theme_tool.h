#ifndef TESSERACT_RVIZ_ENVIRONMENT_PLUGIN_SET_THEME_TOOL_H
#define TESSERACT_RVIZ_ENVIRONMENT_PLUGIN_SET_THEME_TOOL_H

#include <memory>
#include <QObject>

class QAction;

namespace rviz_common
{
class DisplayContext;
}

namespace tesseract_rviz
{
struct SetThemeToolPrivate;
class SetThemeTool : public QObject
{
  Q_OBJECT
public:
  SetThemeTool();
  ~SetThemeTool();
  SetThemeTool(const SetThemeTool&) = delete;
  SetThemeTool& operator=(const SetThemeTool&) = delete;
  SetThemeTool(SetThemeTool&&) = delete;
  SetThemeTool& operator=(SetThemeTool&&) = delete;

  bool isInitialized() const;
  void initialized(rviz_common::DisplayContext* context);

  static std::shared_ptr<SetThemeTool> instance();

private Q_SLOTS:
  void onThemeDefaultSelected();
  void onThemeDarkSelected();
  void onThemeLightSelected();

private:
  std::unique_ptr<SetThemeToolPrivate> data_;
};
}  // namespace tesseract_rviz

#endif  // TESSERACT_RVIZ_ENVIRONMENT_PLUGIN_SET_THEME_TOOL_H
