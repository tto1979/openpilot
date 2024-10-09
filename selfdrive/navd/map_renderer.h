#pragma once

#include <memory>

#include <QOpenGLContext>
#include <QMapLibre/Map>
#include <QMapLibre/Settings>
#include <QTimer>
#include <QGeoCoordinate>
#include <QOpenGLBuffer>
#include <QOffscreenSurface>
#include <QOpenGLFunctions>
#include <QOpenGLFramebufferObject>

#include "msgq/visionipc/visionipc_server.h"
#include "cereal/messaging/messaging.h"

struct Pose {
  struct Vec3 {
    float x, y, z;
  };
  Vec3 position;
  Vec3 orientation;
  
  static Pose from_live_pose(const cereal::LivePose::Reader& live_pose) {
    Pose pose;
    pose.position = {live_pose.getOrientationNED().getX(), live_pose.getOrientationNED().getY(), live_pose.getOrientationNED().getZ()};
    pose.orientation = {live_pose.getAngularVelocityDevice().getX(), live_pose.getAngularVelocityDevice().getY(), live_pose.getAngularVelocityDevice().getZ()};
    return pose;
  }
};

class MapRenderer : public QObject {
  Q_OBJECT

public:
  MapRenderer(const QMapLibre::Settings &, bool online=true);
  uint8_t* getImage();
  void update();
  bool loaded();
  ~MapRenderer();
  void updatePose(const Pose& pose);

private:
  std::unique_ptr<QOpenGLContext> ctx;
  std::unique_ptr<QOffscreenSurface> surface;
  std::unique_ptr<QOpenGLFunctions> gl_functions;
  std::unique_ptr<QOpenGLFramebufferObject> fbo;

  std::unique_ptr<VisionIpcServer> vipc_server;
  std::unique_ptr<PubMaster> pm;
  std::unique_ptr<SubMaster> sm;
  void publish(const double render_time, const bool loaded);
  void sendThumbnail(const uint64_t ts, const kj::Array<capnp::byte> &buf);

  QMapLibre::Settings m_settings;
  QScopedPointer<QMapLibre::Map> m_map;

  void initLayers();

  double start_render_t;
  uint32_t frame_id = 0;
  uint64_t last_pose_rendered = 0;
  bool rendering = false;
  bool rendered() {
    return last_pose_rendered == (*sm)["livePose"].getLogMonoTime();
  }

  QTimer* timer;
  bool ever_loaded = false;
  Pose calibrated_pose;

public slots:
  void updatePosition(QMapLibre::Coordinate position, float bearing);
  void updateRoute(QList<QGeoCoordinate> coordinates);
  void msgUpdate();
};
