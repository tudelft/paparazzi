#include "mini_strip.h"
#include "AircraftManager.h"
#include "gcs_utils.h"
#include "dispatcher_ui.h"
#include "mini_strip.h"
#include <QDebug>
#include <QSettings>

const QMap<QString, QString> MiniStrip::mode_colors{
    // fixedwings
    {"HOME", "#ff0000"}, {"NOGPS", "#ff0000"},
    {"MANUAL", "#ffa500"},
    {"AUTO1", "#10f0e0"},
    {"AUTO2", "#00ff00"},

    // rotorcrafts
    {"KILL", "#ff0000"}, {"SAFE", "#ff0000"}, /*HOME, */

    {"RATE", "#ffa500"}, {"ATT", "#ffa500"},
    {"R_RCC", "#ffa500"},{"A_RCC", "#ffa500"},{"ATT_C", "#ffa500"},{"R_ZH", "#ffa500"},

    {"HOVER", "#10f0e0"}, {"HOV_C", "#10f0e0"}, {"H_ZH", "#10f0e0"},
    {"RC_D", "#10f0e0"}, {"CF", "#10f0e0"}, {"FWD", "#10f0e0"},
    {"MODULE", "#10f0e0"}, {"FLIP", "#10f0e0"}, {"A_ZH", "#10f0e0"},

    {"NAV", "#00ff00"}, {"GUIDED", "#70c070"},
};

MiniStrip::MiniStrip(QString ac_id, QWidget *parent) : QWidget(parent),
    ac_id(ac_id), icons_size(QSize(30, 30)), time_mode(TimeMode::FLIGHT_TIME),
    alt_mode(true), speed_mode(SpeedMode::GROUND_SPEED)
{
    auto gl = new QGridLayout(this);
    auto ac = AircraftManager::get()->getAircraft(ac_id);
    auto ac_name = new QLabel(ac->name());
    ac_name->setStyleSheet("font-weight: bold");
    gl->addWidget(ac_name, 0, 0);


    /////////// alt ///////////
    QHBoxLayout* alt_layout = new QHBoxLayout();
    alt_button = new QToolButton(this);
    alt_button->setIconSize(icons_size);
    agl_icon = QIcon(":/pictures/agl.svg");
    msl_icon = QIcon(":/pictures/msl.svg");
    alt_button->setIcon(agl_icon);
    alt_layout->addWidget(alt_button);
    alt_label = new QLabel("0 m", this);
    alt_label->setToolTip("Altitude AGL");
    alt_layout->addWidget(alt_label);
    gl->addLayout(alt_layout, 1, 1);
    connect(alt_button, &QToolButton::clicked, this,
        [=]() {
            alt_mode = !alt_mode;
            if(alt_mode) {
                alt_button->setIcon(agl_icon);
                alt_label->setToolTip("Altitude AGL");
            } else {
                alt_button->setIcon(msl_icon);
                alt_label->setToolTip("Altitude MSL");
            }
            updateData();
        });

    /////////// flight time ///////////
    QHBoxLayout* ft_lay = new QHBoxLayout();
    time_button = new QToolButton(this);
    time_button->setIconSize(icons_size);
    time_flight = QIcon(":/pictures/flight_time.svg");
    time_block = QIcon(":/pictures/block_time.svg");
    time_stage = QIcon(":/pictures/stage_time.svg");
    time_button->setIcon(time_flight);
    time_button->setToolTip("Flight time");
    ft_lay->addWidget(time_button);
    flight_time_label = new QLabel("0 s", this);
    flight_time_label->setToolTip("Flight time");
    ft_lay->addWidget(flight_time_label);
    ft_lay->addStretch();
    gl->addLayout(ft_lay, 0, 1);
    connect(time_button, &QToolButton::clicked, this,
        [=]() {
        switch (time_mode) {
        case FLIGHT_TIME:
            time_mode = BLOCK_TIME;
            time_button->setToolTip("Block time");
            time_button->setIcon(time_block);
            flight_time_label->setToolTip("Block time");
            break;
        case BLOCK_TIME:
            time_mode = STAGE_TIME;
            time_button->setToolTip("Stage time");
            time_button->setIcon(time_stage);
            flight_time_label->setToolTip("Stage time");
            break;
        case STAGE_TIME:
            time_mode = FLIGHT_TIME;
            time_button->setToolTip("Flight time");
            time_button->setIcon(time_flight);
            flight_time_label->setToolTip("Flight time");
            break;
        }
        updateData();
        });


    /////////// speed //////////////////
    QHBoxLayout* speed_lay = new QHBoxLayout();
    speed_button = new QToolButton(this);
    speed_button->setIconSize(icons_size);
    ground_speed_icon = QIcon(":/pictures/ground_speed.svg");
    air_speed_icon = QIcon(":/pictures/airspeed.svg");
    vertical_speed_icon = QIcon(":/pictures/vertical_speed.svg");
    speed_button->setIcon(ground_speed_icon);
    speed_lay->addWidget(speed_button);
    speed_label = new QLabel("0 m/s", this);
    speed_label->setToolTip("Ground speed");
    speed_lay->addWidget(speed_label);
    gl->addLayout(speed_lay, 2, 1);
    connect(speed_button, &QToolButton::clicked, this,
        [=]() {
            switch (speed_mode) {
            case GROUND_SPEED:
                speed_mode = AIR_SPEED;
                speed_button->setIcon(air_speed_icon);
                speed_label->setToolTip("Air speed");
                break;
            case AIR_SPEED:
                speed_mode = VERTICAL_SPEED;
                speed_button->setIcon(vertical_speed_icon);
                speed_label->setToolTip("Vertical speed");
                break;
            case VERTICAL_SPEED:
                speed_mode = GROUND_SPEED;
                speed_button->setIcon(ground_speed_icon);
                speed_label->setToolTip("Ground speed");
                break;
            }
            updateData();
        });



    /////////// block /////////
    auto block_lay = new QHBoxLayout();
    block_icon = new QLabel(this);
    block_label = new QLabel(this);
    block_lay->addWidget(block_icon);
    block_lay->addWidget(block_label);
    block_lay->addStretch();
    block_icon->setToolTip("block");
    block_label->setToolTip("block");
    gl->addLayout(block_lay, 0, 2);

    ////////// bat /////////////
    auto bat_lay = new QHBoxLayout();
    bat_ok = QIcon(":/pictures/bat_ok.svg");
    bat_low = QIcon(":/pictures/bat_low.svg");
    bat_critic = QIcon(":/pictures/bat_critic.svg");
    bat_catastrophic = QIcon(":/pictures/bat_catastrophic.svg");
    bat_icon = new QLabel(this);
    bat_icon->setPixmap(bat_catastrophic.pixmap(icons_size));
    bat_label = new QLabel("00.0 V", this);
    bat_lay->addWidget(bat_icon);
    bat_lay->addWidget(bat_label);
    gl->addLayout(bat_lay, 1, 0);

    auto bnbc = ac->getAirframe()->getDefine("BAT_NB_CELLS");
    bat_nb_cells = bnbc.has_value() ? bnbc->value.toInt() : 0;


    ////////// throttle /////////////
    auto th_lay = new QHBoxLayout();
    throttle_on = QIcon(":/pictures/throttle_on.svg");
    throttle_killed = QIcon(":/pictures/throttle_killed.svg");
    throttle_icon = new QLabel(this);
    throttle_icon->setPixmap(throttle_killed.pixmap(icons_size));
    th_lay->addWidget(throttle_icon);
    throttle_label = new QLabel("0 %", this);
    throttle_label->setToolTip("Throttle");
    th_lay->addWidget(throttle_label);
    gl->addLayout(th_lay, 2, 0);

    //////// mode //////////
    ap_mode_button = new QPushButton("MODE");
    ap_mode_button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    gl->addWidget(ap_mode_button, 1, 2);
    ap_mode_button->setToolTip("AP mode");
    auto settings = ac->getSettingMenu()->getAllSettings();
    for(auto setting: settings) {
        if(setting->getName() == "autopilot.mode") {
            connect(ap_mode_button, &QPushButton::clicked, this, [=](){
                AircraftManager::get()->getAircraft(ac_id)->setSetting(setting, "AUTO2");
            });
            break;
        }
    }



    ///// Link, GPS, RC //////
    auto misc_lay = new QHBoxLayout();
    link_icon = new QLabel(this);
    link_ok = QIcon(":/pictures/link_ok.svg");
    link_warning = QIcon(":/pictures/link_warning.svg");
    link_lost = QIcon(":/pictures/link_nok.svg");
    link_icon->setPixmap(link_lost.pixmap(icons_size));
    link_icon->setToolTip("Link");
    misc_lay->addWidget(link_icon);

    gps_icon = new QLabel(this);
    gps_ok = QIcon(":/pictures/gps_ok.svg");
    gps_lost = QIcon(":/pictures/gps_nok.svg");
    gps_icon->setPixmap(gps_lost.pixmap(30,30));
    gps_icon->setToolTip("Gps");
    misc_lay->addWidget(gps_icon);


    rc_icon = new QLabel(this);
    rc_ok = QIcon(":/pictures/rc_ok.svg");
    rc_lost = QIcon(":/pictures/rc_nok.svg");
    rc_icon->setPixmap(rc_lost.pixmap(icons_size));
    rc_icon->setToolTip("RC");
    misc_lay->addWidget(rc_icon);

    imu_icon = new QLabel(this);
    imu_ok = QIcon(":/pictures/imu_ok.svg");
    imu_lost = QIcon(":/pictures/imu_nok.svg");
    imu_icon->setPixmap(imu_lost.pixmap(icons_size));
    imu_icon->setToolTip("IMU");
    misc_lay->addWidget(imu_icon);

    gl->addLayout(misc_lay, 2, 2);

    setStyleSheet("QLabel {font-family: \"Monospace\";}");

    auto ac_status = AircraftManager::get()->getAircraft(ac_id)->getStatus();

    connect(ac_status, &AircraftStatus::engine_status, this, &MiniStrip::updateData);
    connect(ac_status, &AircraftStatus::flight_param, this, &MiniStrip::updateData);
    connect(ac_status, &AircraftStatus::telemetry_status, this, &MiniStrip::updateData);
    connect(ac_status, &AircraftStatus::fly_by_wire, this, &MiniStrip::updateData);
    connect(ac_status, &AircraftStatus::ap_status, this, &MiniStrip::updateData);
    connect(ac_status, &AircraftStatus::nav_status, this, &MiniStrip::updateData);
    connect(ac_status->getWatcher(), &AircraftWatcher::bat_status, this, &MiniStrip::handle_bat_status);
}

void MiniStrip::handle_bat_status(AircraftWatcher::BatStatus bs) {
    // change battery icon
    switch (bs) {
    case AircraftWatcher::BatStatus::CATASTROPHIC:
        bat_icon->setPixmap(bat_catastrophic.pixmap(icons_size));
        break;
    case AircraftWatcher::BatStatus::CRITIC:
        bat_icon->setPixmap(bat_critic.pixmap(icons_size));
        break;
    case AircraftWatcher::BatStatus::LOW:
        bat_icon->setPixmap(bat_low.pixmap(icons_size));
        break;
    case AircraftWatcher::BatStatus::OK:
        bat_icon->setPixmap(bat_ok.pixmap(icons_size));
        break;
    }
}

void MiniStrip::updateFlightTime(uint32_t flight_time, uint32_t block_time, uint32_t stage_time) {
    uint32_t time = 0;
    switch (time_mode) {
    case FLIGHT_TIME:
        time = flight_time; break;
    case BLOCK_TIME:
        time = block_time; break;
    case STAGE_TIME:
        time = stage_time; break;

    }
    int hours = static_cast<int>(static_cast<int64_t>(time)/3600);
    int minutes = static_cast<int>(static_cast<int64_t>(time)/60 - hours*60);
    int seconds = static_cast<int>(static_cast<int64_t>(time) - minutes*60 -hours*3600);

    QString f_time = QString("%1").arg(hours, 2, 10, QChar('0')) + ":" +
                     QString("%1").arg(minutes, 2, 10, QChar('0')) + ":" +
                     QString("%1").arg(seconds, 2, 10, QChar('0'));
    flight_time_label->setText(f_time);
}

void MiniStrip::updateAp(QString ap_mode) {
    QColor color = QColor(Qt::white);
    if(mode_colors.contains(ap_mode)) {
        color = QColor(mode_colors[ap_mode]);

    }
    QString style_back = "background: rgb(%1, %2, %3);";
    QString style = "QPushButton {";
    style += style_back.arg(color.red()).arg(color.green()).arg(color.blue());
    style += "color:black; font-size:12px;";
    style += " font-weight:bold;";
    style += "}";
    ap_mode_button->setStyleSheet(style);
    ap_mode_button->setText(ap_mode);
}

void MiniStrip::updateGps(QString gps_mode) {
    if(gps_mode == "NOFIX" || gps_mode == "NA" || gps_mode == "2D") {
        gps_icon->setPixmap(gps_lost.pixmap(30,30));
    } else {
        gps_icon->setPixmap(gps_ok.pixmap(30,30));
    }
    gps_icon->setToolTip("Gps " + QString(gps_mode));
}

void MiniStrip::updateImu(QString state_filter_mode) {
    if(state_filter_mode == "OK") {
        imu_icon->setPixmap(imu_ok.pixmap(icons_size));
    } else {
        imu_icon->setPixmap(imu_lost.pixmap(icons_size));
    }
    imu_icon->setToolTip("IMU " + QString(state_filter_mode));
}

void MiniStrip::updateCurrentBlock(uint8_t cur_block) {
    auto block = AircraftManager::get()->getAircraft(ac_id)->getFlightPlan()->getBlock(cur_block);
    QString block_name = block->getName();
    block_label->setToolTip("Block " + block_name);
    if(block_name.size() > 15) {
        block_name.truncate(12);
        block_name += "...";
    }
    block_label->setText(block_name);

    auto settings = getAppSettings();

    QString icon_name = block->getIcon();
    if(icon_name != "") {
        QString icon_path = appConfig()->value("GCS_ICONS_PATH").toString() + "/" + icon_name;
        block_icon->setPixmap(QIcon(icon_path).pixmap(icons_size));
    } else {
        block_icon->setText(QString::fromUtf8("\xE2\x88\x85"));
    }
}

void MiniStrip::updateData() {

    // ENGINE_STATUS
    float bat = 0;
    float throttle = 0;

    // AP_STATUS
    QString kill_mode, ap_mode, gps_mode, state_filter_mode; // lat_mode, horiz_mode, gaz_mode;
    uint32_t flight_time = 0;

    // FLIGHT_PARAM
    float speed = 0;
    float alt = 0;
    float climb = 0;
    float agl = 0;
    float airspeed = 0;

    // FLY_BY_WIRE
    QString rc_status, rc_mode;

    // NAV_STATUS
    uint8_t cur_block = 0;
    float target_alt = 0;
    uint32_t block_time = 0;
    uint32_t stage_time = 0;


    auto engine_status_msg = AircraftManager::get()->getAircraft(ac_id)->getStatus()->getMessage("ENGINE_STATUS");
    if(engine_status_msg) {
        engine_status_msg->getField("throttle", throttle);
        engine_status_msg->getField("bat", bat);
    }

    auto ap_status_msg = AircraftManager::get()->getAircraft(ac_id)->getStatus()->getMessage("AP_STATUS");
    if(ap_status_msg) {
        ap_status_msg->getField("flight_time", flight_time);
        ap_status_msg->getField("kill_mode", kill_mode);
        ap_status_msg->getField("ap_mode", ap_mode);
        ap_status_msg->getField("gps_mode", gps_mode);
        ap_status_msg->getField("state_filter_mode", state_filter_mode);
    }

    auto flight_param_msg = AircraftManager::get()->getAircraft(ac_id)->getStatus()->getMessage("FLIGHT_PARAM");
    if(flight_param_msg) {

        flight_param_msg->getField("speed", speed);
        flight_param_msg->getField("alt", alt);
        flight_param_msg->getField("climb", climb);
        flight_param_msg->getField("agl", agl);
        flight_param_msg->getField("airspeed", airspeed);
    }

    auto telemetry_messages = AircraftManager::get()->getAircraft(ac_id)->getStatus()->getTelemetryMessages();

    auto fly_by_wire_msg = AircraftManager::get()->getAircraft(ac_id)->getStatus()->getMessage("FLY_BY_WIRE");
    if(fly_by_wire_msg) {
        fly_by_wire_msg->getField("rc_status", rc_status);
        fly_by_wire_msg->getField("rc_mode", rc_mode);
    }

    auto nav_status_msg = AircraftManager::get()->getAircraft(ac_id)->getStatus()->getMessage("NAV_STATUS");
    if(nav_status_msg) {
        nav_status_msg->getField("cur_block", cur_block);
        nav_status_msg->getField("target_alt", target_alt);
        nav_status_msg->getField("block_time", block_time);
        nav_status_msg->getField("stage_time", stage_time);
    }


    // batterie
    if(bat_nb_cells != 0) {
        bat_label->setText(QString("%1 V/c").arg(bat/bat_nb_cells, 5, 'f', 2));
        bat_label->setToolTip(QString("%1 V").arg(bat, 5, 'f', 2));
        bat_icon->setToolTip(QString("%1 V").arg(bat, 5, 'f', 2));
    } else {
        bat_label->setText(QString("%1 V").arg(bat, 5, 'f', 2));
    }

    // throttle
    /// TODO: change bat icon : full/half/low
    throttle_label->setText(QString("%1 %").arg(throttle, 4, 'f', 1));

    // kill
    if(kill_mode == "OFF") {
        throttle_icon->setPixmap(throttle_on.pixmap(icons_size));
    } else {
        throttle_icon->setPixmap(throttle_killed.pixmap(icons_size));
    }

    // flight time
    updateFlightTime(flight_time, block_time, stage_time);


    // mode
    updateAp(ap_mode);

    // GPS
    updateGps(gps_mode);

    // IMU
    updateImu(state_filter_mode);

    // link
    QString tip_txt;
    int nb_links_ok = 0;
    for(auto &telemetry_msg: telemetry_messages) {
        float time_since_last_msg = 9999;
        telemetry_msg.getField("time_since_last_msg", time_since_last_msg);
        if(time_since_last_msg <= 5) {
            nb_links_ok += 1;
        }
    }


    if(nb_links_ok == telemetry_messages.size()) {
        // all links ok
        link_icon->setPixmap(link_ok.pixmap(icons_size));
    } else if(nb_links_ok > 0) {
        // some links lost
        link_icon->setPixmap(link_warning.pixmap(icons_size));
    } else {
        // All links lost
        link_icon->setPixmap(link_lost.pixmap(icons_size));
    }

    if(telemetry_messages.size() > 1) {
        // multiple links
        tip_txt = QString("Link %1 / %2").arg(nb_links_ok).arg(telemetry_messages.size());
    } else {
        // one link
        if(nb_links_ok > 0) {
            tip_txt = "Link ok";
        } else {
            tip_txt = "Link lost";
        }
    }
    link_icon->setToolTip(tip_txt);

    // RC
    if(rc_status == "OK") {
        rc_icon->setPixmap(rc_ok.pixmap(icons_size));
    } else {
        rc_icon->setPixmap(rc_lost.pixmap(icons_size));
    }
    rc_icon->setToolTip("RC " + rc_status);

    // alt
    double alt_display = alt_mode ? agl: alt;
    double diff_alt = alt - target_alt;
    QString alt_str = QString("%1m | %2%3 m")
            .arg(alt_display, 3, 'f', 0)
            .arg(diff_alt >= 0 ? "+": "")
            .arg(diff_alt, 3, 'f', abs(diff_alt) > 10 ? 0: 1);
    alt_label->setText(alt_str);

    // speed
    switch (speed_mode) {
    case GROUND_SPEED:
        speed_label->setText(QString::number(speed, 'f', 1) + " m/s");
        break;
    case AIR_SPEED:
        speed_label->setText(QString::number(airspeed, 'f', 1) + " m/s");
        break;
    case VERTICAL_SPEED:
        speed_label->setText(QString::number(climb, 'f', 1) + " m/s");
        break;
    }

    // current block
    updateCurrentBlock(cur_block);


//    QString txt = QString::number(climb, 'f', 1);
//    if(climb > 0) {
//        txt = "+" + txt;
//    }
//    full_alt_graph->setSecondayText(txt);
//    short_vspeed_label->setText(txt);
//    if(abs(speed) > 0.1) {
//        full_alt_graph->setIndicatorAngle((climb/speed)*1.2);
//    } else if(abs(climb) > 0.01){
//        full_alt_graph->setIndicatorAngle(climb/abs(climb)*0.3);
//    } else {
//        full_alt_graph->setIndicatorAngle(0);
//    }

//    if(climb > 0.5) {
//        short_vspeed_indicator->setText(QString::fromUtf8("\xE2\x86\x97"));
//    } else if(climb < -0.5) {
//        short_vspeed_indicator->setText(QString::fromUtf8("\xE2\x86\x98"));
//    } else {
//        short_vspeed_indicator->setText(QString::fromUtf8("\xE2\x86\x92"));
//    }


    emit updated();

}

bool MiniStrip::eventFilter(QObject *object, QEvent *event)
{
    (void) object;
    if (event->type() == QEvent::MouseButtonPress) {
        emit DispatcherUi::get()->ac_selected(ac_id);
        return true;
    }
    return false;
}
