#pragma once

#include <QVBoxLayout>
#include <memory>
#include <utility>
#include <vector>

#include "selfdrive/ui/qt/onroad/hud.h"
#include "selfdrive/ui/qt/onroad/buttons.h"
#include "selfdrive/ui/qt/onroad/driver_monitoring.h"
#include "selfdrive/ui/qt/widgets/cameraview.h"

class AnnotatedCameraWidget : public CameraWidget {
  Q_OBJECT

public:
  explicit AnnotatedCameraWidget(VisionStreamType type, QWidget* parent = 0);
  void updateState(const UIState &s);

private:
  void drawCenteredText(QPainter &p, int x, int y, const QString &text, QColor color);

  QVBoxLayout *main_layout;
  ExperimentalButton *experimental_btn;
  DriverMonitorRenderer dmon;
  HudRenderer hud;
  std::unique_ptr<PubMaster> pm;

  int skip_frame_count = 0;
  int status = STATUS_DISENGAGED;
  bool hideBottomIcons = false;
  bool wide_cam_requested = false;

protected:
  void paintGL() override;
  void initializeGL() override;
  void showEvent(QShowEvent *event) override;
  mat4 calcFrameMatrix() override;
  void drawLaneLines(QPainter &painter, const UIState *s);
  void drawLead(QPainter &painter, const cereal::RadarState::LeadData::Reader &lead_data, const QPointF &vd , int num);
  void drawLockon(QPainter &painter, const cereal::ModelDataV2::LeadDataV3::Reader &lead_data, const QPointF &vd , int num  /*不使用, size_t leads_num , const cereal::RadarState::LeadData::Reader &lead0, const cereal::RadarState::LeadData::Reader &lead1 */);

  double prev_draw_t = 0;
  FirstOrderFilter fps_filter;
};
