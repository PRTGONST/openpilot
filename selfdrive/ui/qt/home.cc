#include "selfdrive/ui/qt/home.h"

#include <QDateTime>
#include <QHBoxLayout>
#include <QMouseEvent>
#include <QVBoxLayout>

#include "selfdrive/common/params.h"
#include "selfdrive/common/swaglog.h"
#include "selfdrive/common/timing.h"
#include "selfdrive/common/util.h"
#include "selfdrive/ui/qt/widgets/drive_stats.h"
#include "selfdrive/ui/qt/widgets/setup.h"

// HomeWindow: the container for the offroad and onroad UIs

HomeWindow::HomeWindow(QWidget* parent) : QWidget(parent) {
  QHBoxLayout *layout = new QHBoxLayout(this);
  layout->setMargin(0);
  layout->setSpacing(0);

  sidebar = new Sidebar(this);
  layout->addWidget(sidebar);
  QObject::connect(this, &HomeWindow::update, sidebar, &Sidebar::update);
  QObject::connect(sidebar, &Sidebar::openSettings, this, &HomeWindow::openSettings);

  slayout = new QStackedLayout();
  layout->addLayout(slayout);

  onroad = new OnroadWindow(this);
  slayout->addWidget(onroad);
  QObject::connect(this, &HomeWindow::update, onroad, &OnroadWindow::update);
  QObject::connect(this, &HomeWindow::offroadTransitionSignal, onroad, &OnroadWindow::offroadTransition);

  home = new OffroadHome();
  slayout->addWidget(home);
  QObject::connect(this, &HomeWindow::openSettings, home, &OffroadHome::refresh);

  setLayout(layout);
}

void HomeWindow::offroadTransition(bool offroad) {
  if (offroad) {
    slayout->setCurrentWidget(home);
  } else {
    slayout->setCurrentWidget(onroad);
  }
  sidebar->setVisible(offroad);
  emit offroadTransitionSignal(offroad);
}

void HomeWindow::mousePressEvent(QMouseEvent* e) {
  // TODO: make a nice driver view widget
  if (QUIState::ui_state.scene.driver_view) {
    Params().putBool("IsDriverViewEnabled", false);
    QUIState::ui_state.scene.driver_view = false;
    return;
  }

  // Toggle speed limit control enabled
  Rect touch_rect = QUIState::ui_state.speed_limit_sign_touch_rect;
  Rect debug_tap_rect = QUIState::ui_state.debug_tap_rect;
  if (sidebar->isVisible()) {
    touch_rect.x += sidebar->width();
    debug_tap_rect.x += sidebar->width();
  }
  if (QUIState::ui_state.scene.controls_state.getSpeedLimit() > 0.0 && touch_rect.ptInRect(e->x(), e->y())) {
    // If touching the speed limit sign area when visible
    QUIState::ui_state.last_speed_limit_sign_tap = seconds_since_boot();
    QUIState::ui_state.scene.speed_limit_control_enabled = !QUIState::ui_state.scene.speed_limit_control_enabled;
    Params().putBool("SpeedLimitControl", QUIState::ui_state.scene.speed_limit_control_enabled);
  }

  // Issue a debug print.
  else if (debug_tap_rect.ptInRect(e->x(), e->y())) {
    char param_name[64] = {'\0'};
    time_t rawtime = time(NULL);
    struct tm timeinfo;
    localtime_r(&rawtime, &timeinfo);
    strftime(param_name, sizeof(param_name), "%Y-%m-%d--%H-%M-%S", &timeinfo);

    auto live_map_data = QUIState::ui_state.scene.live_map_data;
    char s[512];
    int size = snprintf(s, sizeof(s), "Datetime: %s\nPos, Bearing: (%.6f, %.6f), %.2f; Speed: %.1f\n"
      "sl: %.1f, valid: %d\nsl_ahead: %.1f, valid: %d, dist: %.1f\ntsl: %.1f, valid: %d, end dist: %.1f\n"
      "tsl_ahead: %.1f, valid: %d, dist: %.1f\n\nSPEED LIMIT CONTROLLER:\nsl: %.1f, state: %hu\n\n"
      "TURN SPEED CONTROLLER:\nspeed: %.1f, state: %hu\n\nTURN CONTROLLER:\nacc: %.3f, state: %hu", 
      param_name, live_map_data.getLastGpsLatitude(), live_map_data.getLastGpsLongitude(),
      live_map_data.getLastGpsLBearingDeg(), live_map_data.getLastGpsSpeed(), live_map_data.getSpeedLimit() * 3.6, 
      live_map_data.getSpeedLimitValid(), live_map_data.getSpeedLimitAhead() * 3.6,
      live_map_data.getSpeedLimitAheadValid(), live_map_data.getSpeedLimitAheadDistance(),
      live_map_data.getTurnSpeedLimit() * 3.6, live_map_data.getTurnSpeedLimitValid(),
      live_map_data.getTurnSpeedLimitEndDistance(), live_map_data.getTurnSpeedLimitAhead() * 3.6, 
      live_map_data.getTurnSpeedLimitAheadValid(), live_map_data.getTurnSpeedLimitAheadDistance(), 
      QUIState::ui_state.scene.controls_state.getSpeedLimit(),
      QUIState::ui_state.scene.controls_state.getSpeedLimitControlState(),
      QUIState::ui_state.scene.controls_state.getTurnSpeed(),
      QUIState::ui_state.scene.controls_state.getTurnSpeedControlState(), 
      QUIState::ui_state.scene.controls_state.getTurnAcc(),
      QUIState::ui_state.scene.controls_state.getTurnControllerState());

    Params().put(param_name, s, size < sizeof(s) ? size : sizeof(s));
    SubMaster &sm = *(QUIState::ui_state.sm);
    QUIState::ui_state.scene.display_debug_alert_frame = sm.frame;
  }

  // Handle sidebar collapsing
  else if (onroad->isVisible() && (!sidebar->isVisible() || e->x() > sidebar->width())) {
    sidebar->setVisible(!sidebar->isVisible());
  }
}

// OffroadHome: the offroad home page

OffroadHome::OffroadHome(QWidget* parent) : QFrame(parent) {
  QVBoxLayout* main_layout = new QVBoxLayout();
  main_layout->setMargin(50);

  // top header
  QHBoxLayout* header_layout = new QHBoxLayout();

  date = new QLabel();
  date->setStyleSheet(R"(font-size: 55px;)");
  header_layout->addWidget(date, 0, Qt::AlignHCenter | Qt::AlignLeft);

  alert_notification = new QPushButton();
  alert_notification->setVisible(false);
  QObject::connect(alert_notification, &QPushButton::released, this, &OffroadHome::openAlerts);
  header_layout->addWidget(alert_notification, 0, Qt::AlignHCenter | Qt::AlignRight);

  std::string brand = Params().getBool("Passive") ? "dashcam" : "openpilot";
  QLabel* version = new QLabel(QString::fromStdString(brand + " v" + Params().get("Version")));
  version->setStyleSheet(R"(font-size: 55px;)");
  header_layout->addWidget(version, 0, Qt::AlignHCenter | Qt::AlignRight);

  main_layout->addLayout(header_layout);

  // main content
  main_layout->addSpacing(25);
  center_layout = new QStackedLayout();

  QHBoxLayout* statsAndSetup = new QHBoxLayout();
  statsAndSetup->setMargin(0);

  DriveStats* drive = new DriveStats;
  drive->setFixedSize(800, 800);
  statsAndSetup->addWidget(drive);

  SetupWidget* setup = new SetupWidget;
  statsAndSetup->addWidget(setup);

  QWidget* statsAndSetupWidget = new QWidget();
  statsAndSetupWidget->setLayout(statsAndSetup);

  center_layout->addWidget(statsAndSetupWidget);

  alerts_widget = new OffroadAlert();
  QObject::connect(alerts_widget, &OffroadAlert::closeAlerts, this, &OffroadHome::closeAlerts);
  center_layout->addWidget(alerts_widget);
  center_layout->setAlignment(alerts_widget, Qt::AlignCenter);

  main_layout->addLayout(center_layout, 1);

  // set up refresh timer
  timer = new QTimer(this);
  QObject::connect(timer, &QTimer::timeout, this, &OffroadHome::refresh);
  timer->start(10 * 1000);

  setLayout(main_layout);
  setStyleSheet(R"(
    OffroadHome {
      background-color: black;
    }
    * {
     color: white;
    }
  )");
}

void OffroadHome::showEvent(QShowEvent *event) {
  refresh();
}

void OffroadHome::openAlerts() {
  center_layout->setCurrentIndex(1);
}

void OffroadHome::closeAlerts() {
  center_layout->setCurrentIndex(0);
}

void OffroadHome::refresh() {
  bool first_refresh = !date->text().size();
  if (!isVisible() && !first_refresh) {
    return;
  }

  date->setText(QDateTime::currentDateTime().toString("dddd, MMMM d"));

  // update alerts

  alerts_widget->refresh();
  if (!alerts_widget->alertCount && !alerts_widget->updateAvailable) {
    emit closeAlerts();
    alert_notification->setVisible(false);
    return;
  }

  if (alerts_widget->updateAvailable) {
    alert_notification->setText("UPDATE");
  } else {
    int alerts = alerts_widget->alertCount;
    alert_notification->setText(QString::number(alerts) + " ALERT" + (alerts == 1 ? "" : "S"));
  }

  if (!alert_notification->isVisible() && !first_refresh) {
    emit openAlerts();
  }
  alert_notification->setVisible(true);

  // Red background for alerts, blue for update available
  QString style = QString(R"(
    padding: 15px;
    padding-left: 30px;
    padding-right: 30px;
    border: 1px solid;
    border-radius: 5px;
    font-size: 40px;
    font-weight: 500;
    background-color: #E22C2C;
  )");
  if (alerts_widget->updateAvailable) {
    style.replace("#E22C2C", "#364DEF");
  }
  alert_notification->setStyleSheet(style);
}
