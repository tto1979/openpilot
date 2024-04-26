#include "map_settings.h"

#include <QApplication>
#include <QDebug>

#include "common/util.h"
#include "selfdrive/ui/qt/request_repeater.h"
#include "selfdrive/ui/qt/widgets/scrollview.h"

static QString shorten(const QString &str, int max_len) {
  return str.size() > max_len ? str.left(max_len).trimmed() + "…" : str;
}

MapSettings::MapSettings(bool closeable, QWidget *parent) : QFrame(parent) {
  close_icon = loadPixmap("../assets/icons/close.svg", {100, 100});
  setContentsMargins(0, 0, 0, 0);

  auto *frame = new QVBoxLayout(this);
  frame->setContentsMargins(40, 40, 40, 0);
  frame->setSpacing(0);

  auto *heading_frame = new QHBoxLayout;
  heading_frame->setContentsMargins(0, 0, 0, 0);
  heading_frame->setSpacing(32);
  {
    if (closeable) {
      auto *close_btn = new QPushButton("←");
      close_btn->setStyleSheet(R"(
        QPushButton {
          color: #FFFFFF;
          font-size: 100px;
          padding-bottom: 8px;
          border 1px grey solid;
          border-radius: 70px;
          background-color: #292929;
          font-weight: 500;
        }
        QPushButton:pressed {
          background-color: #3B3B3B;
        }
      )");
      close_btn->setFixedSize(140, 140);
      QObject::connect(close_btn, &QPushButton::clicked, [=]() { emit closeSettings(); });
      // TODO: read map_on_left from ui state
      heading_frame->addWidget(close_btn);
    }

    auto *heading = new QVBoxLayout;
    heading->setContentsMargins(0, 0, 0, 0);
    heading->setSpacing(16);
    {
      auto *title = new QLabel(tr("NAVIGATION"), this);
      title->setStyleSheet("color: #FFFFFF; font-size: 54px; font-weight: 600;");
      heading->addWidget(title);

      auto *subtitle = new QLabel(tr("Manage at connect.comma.ai"), this);
      subtitle->setStyleSheet("color: #A0A0A0; font-size: 40px; font-weight: 300;");
      heading->addWidget(subtitle);
    }
    heading_frame->addLayout(heading, 1);
  }
  frame->addLayout(heading_frame);
  frame->addSpacing(32);

  current_widget = new DestinationWidget(this);
  QObject::connect(current_widget, &DestinationWidget::actionClicked, [=]() {
    if (current_destination.empty()) return;
    params.remove("NavDestination");
    updateCurrentRoute();
  });
  frame->addWidget(current_widget);
  frame->addSpacing(32);

  QWidget *destinations_container = new QWidget(this);
  destinations_layout = new QVBoxLayout(destinations_container);
  destinations_layout->setContentsMargins(0, 32, 0, 32);
  destinations_layout->setSpacing(20);
  ScrollView *destinations_scroller = new ScrollView(destinations_container, this);
  destinations_scroller->setFrameShape(QFrame::NoFrame);
  frame->addWidget(destinations_scroller);

  setStyleSheet("MapSettings { background-color: #333333; }");

  QObject::connect(NavigationRequest::instance(), &NavigationRequest::locationsUpdated, this, &MapSettings::updateLocations);
  QObject::connect(NavigationRequest::instance(), &NavigationRequest::nextDestinationUpdated, this, &MapSettings::updateCurrentRoute);

  current_locations = NavigationRequest::instance()->currentLocations();
}

void MapSettings::mousePressEvent(QMouseEvent *ev) {
  // Prevent mouse event from propagating up
  ev->accept();
}

void MapSettings::showEvent(QShowEvent *event) {
  updateCurrentRoute();
}

void MapSettings::updateCurrentRoute() {
  auto dest = QString::fromStdString(params.get("NavDestination"));
  if (dest.size()) {
    QJsonDocument doc = QJsonDocument::fromJson(dest.trimmed().toUtf8());
    if (doc.isNull()) {
      qWarning() << "JSON Parse failed on NavDestination" << dest;
      return;
    }
    current_destination = doc.object();
    current_widget->set(current_destination, true);
  } else {
    current_destination = {};
    current_widget->unset("", true);
  }
  if (isVisible()) refresh();
}

void MapSettings::updateLocations(const QJsonArray &locations) {
  current_locations = locations;
  refresh();
}

void MapSettings::refresh() {
  setUpdatesEnabled(false);
  // TODO: should we build a new layout and swap it in?
  clearLayout(destinations_layout);

  bool has_home = false, has_work = false;
  for (auto location : current_locations) {
    auto dest = location.toObject();
    if (dest["save_type"].toString() == NAV_TYPE_FAVORITE) {
      has_home = has_home || dest["label"].toString() == NAV_FAVORITE_LABEL_HOME;
      has_work = has_work || dest["label"].toString() == NAV_FAVORITE_LABEL_WORK;
    }
    if (dest == current_destination) continue;

    auto widget = new DestinationWidget(this);
    widget->set(dest, false);
    QObject::connect(widget, &QPushButton::clicked, [this, dest]() {
      navigateTo(dest);
      emit closeSettings();
    });

    destinations_layout->addWidget(widget);
  }

  // add home and work if missing
  if (!has_home) {
    auto widget = new DestinationWidget(this);
    widget->unset(NAV_FAVORITE_LABEL_HOME);
    destinations_layout->insertWidget(0, widget);
  }
  if (!has_work) {
    auto widget = new DestinationWidget(this);
    widget->unset(NAV_FAVORITE_LABEL_WORK);
    // TODO: refactor to remove this hack
    int index = !has_home || (current_destination["save_type"] == NAV_TYPE_FAVORITE && current_destination["label"] == NAV_FAVORITE_LABEL_HOME) ? 0 : 1;
    destinations_layout->insertWidget(index, widget);
  }

  destinations_layout->addStretch();
  setUpdatesEnabled(true);
}

void MapSettings::navigateTo(const QJsonObject &place) {
  QJsonDocument doc(place);
  params.put("NavDestination", doc.toJson().toStdString());
  updateCurrentRoute();
}

DestinationWidget::DestinationWidget(QWidget *parent) : QPushButton(parent) {
  setContentsMargins(0, 0, 0, 0);

  auto *frame = new QHBoxLayout(this);
  frame->setContentsMargins(32, 24, 32, 24);
  frame->setSpacing(32);

  icon = new QLabel(this);
  icon->setAlignment(Qt::AlignCenter);
  icon->setFixedSize(96, 96);
  icon->setObjectName("icon");
  frame->addWidget(icon);

  auto *inner_frame = new QVBoxLayout;
  inner_frame->setContentsMargins(0, 0, 0, 0);
  inner_frame->setSpacing(0);
  {
    title = new ElidedLabel(this);
    title->setAttribute(Qt::WA_TransparentForMouseEvents);
    inner_frame->addWidget(title);

    subtitle = new ElidedLabel(this);
    subtitle->setAttribute(Qt::WA_TransparentForMouseEvents);
    subtitle->setObjectName("subtitle");
    inner_frame->addWidget(subtitle);
  }
  frame->addLayout(inner_frame, 1);

  action = new QPushButton(this);
  action->setFixedSize(96, 96);
  action->setObjectName("action");
  action->setStyleSheet("font-size: 65px; font-weight: 600;");
  QObject::connect(action, &QPushButton::clicked, [=]() { emit clicked(); });
  QObject::connect(action, &QPushButton::clicked, [=]() { emit actionClicked(); });
  frame->addWidget(action);

  setFixedHeight(164);
  setStyleSheet(R"(
    DestinationWidget { background-color: #202123; border-radius: 10px; }
    QLabel { color: #FFFFFF; font-size: 48px; font-weight: 400; }
    #icon { background-color: #3B4356; border-radius: 48px; }
    #subtitle { color: #9BA0A5; }
    #action { border: none; border-radius: 48px; color: #FFFFFF; padding-bottom: 4px; }

    /* current destination */
    [current="true"] { background-color: #E8E8E8; }
    [current="true"] QLabel { color: #000000; }
    [current="true"] #icon { background-color: #42906B; }
    [current="true"] #subtitle { color: #333333; }
    [current="true"] #action { color: #202123; }

    /* no saved destination */
    [set="false"] QLabel { color: #9BA0A5; }
    [current="true"][set="false"] QLabel { color: #A0000000; }

    /* pressed */
    [current="false"]:pressed { background-color: #18191B; }
    [current="true"] #action:pressed { background-color: #D6D6D6; }
  )");
}

void DestinationWidget::set(const QJsonObject &destination, bool current) {
  setProperty("current", current);
  setProperty("set", true);

  auto icon_pixmap = current ? icons().directions : icons().recent;
  auto title_text = destination["place_name"].toString();
  auto subtitle_text = destination["place_details"].toString();

  if (destination["save_type"] == NAV_TYPE_FAVORITE) {
    if (destination["label"] == NAV_FAVORITE_LABEL_HOME) {
      icon_pixmap = icons().home;
      subtitle_text = title_text + ", " + subtitle_text;
      title_text = tr("Home");
    } else if (destination["label"] == NAV_FAVORITE_LABEL_WORK) {
      icon_pixmap = icons().work;
      subtitle_text = title_text + ", " + subtitle_text;
      title_text = tr("Work");
    } else {
      icon_pixmap = icons().favorite;
    }
  }

  icon->setPixmap(icon_pixmap);

  // TODO: onroad and offroad have different dimensions
  title->setText(shorten(title_text, 26));
  subtitle->setText(shorten(subtitle_text, 26));
  subtitle->setVisible(true);

  // TODO: use pixmap
  action->setAttribute(Qt::WA_TransparentForMouseEvents, !current);
  action->setText(current ? "×" : "→");
  action->setVisible(true);

  setStyleSheet(styleSheet());
}

void DestinationWidget::unset(const QString &label, bool current) {
  setProperty("current", current);
  setProperty("set", false);

  if (label.isEmpty()) {
    icon->setPixmap(icons().directions);
    title->setText(tr("No destination set"));
  } else {
    QString title_text = label == NAV_FAVORITE_LABEL_HOME ? tr("home") : tr("work");
    icon->setPixmap(label == NAV_FAVORITE_LABEL_HOME ? icons().home : icons().work);
    title->setText(tr("No %1 location set").arg(title_text));
  }

  subtitle->setVisible(false);
  action->setVisible(false);

  setStyleSheet(styleSheet());
}

// singleton NavigationRequest

NavigationRequest *NavigationRequest::instance() {
  static NavigationRequest *request = new NavigationRequest(qApp);
  return request;
}

NavigationRequest::NavigationRequest(QObject *parent) : QObject(parent) {
  if (auto dongle_id = getDongleId()) {
    {
      // Fetch favorite and recent locations
      QString url = CommaApi::BASE_URL + "/v1/navigation/" + *dongle_id + "/locations";
      RequestRepeater *repeater = new RequestRepeater(this, url, "ApiCache_NavDestinations", 30, true);
      QObject::connect(repeater, &RequestRepeater::requestDone, this, &NavigationRequest::parseLocationsResponse);
    }
    {
      auto param_watcher = new ParamWatcher(this);
      QObject::connect(param_watcher, &ParamWatcher::paramChanged, this, &NavigationRequest::nextDestinationUpdated);

      // Destination set while offline
      QString url = CommaApi::BASE_URL + "/v1/navigation/" + *dongle_id + "/next";
      HttpRequest *deleter = new HttpRequest(this);
      RequestRepeater *repeater = new RequestRepeater(this, url, "", 10, true);
      QObject::connect(repeater, &RequestRepeater::requestDone, [=](const QString &resp, bool success) {
        if (success && resp != "null") {
          if (params.get("NavDestination").empty()) {
            qWarning() << "Setting NavDestination from /next" << resp;
            params.put("NavDestination", resp.toStdString());
          } else {
            qWarning() << "Got location from /next, but NavDestination already set";
          }
          // Send DELETE to clear destination server side
          deleter->sendRequest(url, HttpRequest::Method::DELETE);
        }

        // athena can set destination at any time
        param_watcher->addParam("NavDestination");
      });
    }
  }
}

static void swap(QJsonValueRef v1, QJsonValueRef v2) { std::swap(v1, v2); }

void NavigationRequest::parseLocationsResponse(const QString &response, bool success) {
  if (!success || response == prev_response) return;

  prev_response = response;
  QJsonDocument doc = QJsonDocument::fromJson(response.trimmed().toUtf8());
  if (doc.isNull()) {
    qWarning() << "JSON Parse failed on navigation locations" << response;
    return;
  }

  // Sort: HOME, WORK, alphabetical FAVORITES, and then most recent (as returned by API)
  locations = doc.array();
  std::stable_sort(locations.begin(), locations.end(), [](const QJsonValue &a, const QJsonValue &b) {
    if (a["save_type"] == NAV_TYPE_FAVORITE || b["save_type"] == NAV_TYPE_FAVORITE) {
      QString a_label = a["label"].toString(), b_label = b["label"].toString();
      return std::tuple(a["save_type"].toString(), (a_label.isEmpty() ? "xxx" : a_label), a["place_name"].toString()) <
             std::tuple(b["save_type"].toString(), (b_label.isEmpty() ? "xxx" : b_label), b["place_name"].toString());
    } else {
      return false;
    }
  });
  emit locationsUpdated(locations);
}
