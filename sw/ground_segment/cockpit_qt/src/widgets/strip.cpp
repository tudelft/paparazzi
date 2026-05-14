#include "strip.h"
#include <QMessageBox>
#include "AircraftManager.h"
#include <QPainter>
#include <QPaintEvent>
#include "dispatcher_ui.h"
#include "units.h"
#include "gcs_utils.h"
#include "globalconfig.h"
#include <QInputDialog>
#include <QTimeEdit>
#include <QDialogButtonBox>

QMap<QString, Strip*> Strip::strips;

Strip::Strip(QString ac_id, QWidget *parent,  bool full) : QWidget(parent), _ac_id(ac_id)
{
    auto mainLayout = new QVBoxLayout(this);
    mainLayout->setSizeConstraint(QLayout::SetFixedSize);
    mainLayout->setContentsMargins(4, 2, 4, 4);

    auto name_label = new QLabel(QString("<b>%1 (%2)</b>").arg(AircraftManager::get()->getAircraft(_ac_id)->name(), _ac_id), this);
    name_label->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    name_label->setStyleSheet("font-weight: bold; border: none; background: transparent;");
    mainLayout->addWidget(name_label);

    auto stripsLayout = new QHBoxLayout();
    stripsLayout->setContentsMargins(0, 0, 0, 0);
    mainLayout->addLayout(stripsLayout);

    build_full_strip();
    build_short_strip();
    stripsLayout->addWidget(full_strip);
    stripsLayout->addWidget(short_strip);

    if(full) {
        short_strip->hide();
    } else {
        full_strip->hide();
    }

    ac_color = AircraftManager::get()->getAircraft(_ac_id)->getColor();
    int hue = ac_color.hue();
    int sat = ac_color.saturation();
    ac_color.setHsv(hue, static_cast<int>(sat*0.2), 255);


    connect(AircraftManager::get()->getAircraft(_ac_id)->getStatus(),
            &AircraftStatus::engine_status, this, &Strip::updateEngineStatus);
    connect(AircraftManager::get()->getAircraft(_ac_id)->getStatus(),
            &AircraftStatus::flight_param, this, &Strip::updateFlightParams);
    connect(AircraftManager::get()->getAircraft(_ac_id)->getStatus(),
            &AircraftStatus::telemetry_status, this, &Strip::updateTelemetryStatus);
    connect(AircraftManager::get()->getAircraft(_ac_id)->getStatus(),
            &AircraftStatus::fly_by_wire, this, &Strip::updateFBW);
    connect(AircraftManager::get()->getAircraft(_ac_id)->getStatus(),
            &AircraftStatus::ap_status, this, &Strip::updateApStatus);

    connect(AircraftManager::get()->getAircraft(_ac_id)->getStatus(),
            &AircraftStatus::nav_status, this, &Strip::updateAltTargetDiff);
    connect(AircraftManager::get()->getAircraft(_ac_id)->getStatus(),
            &AircraftStatus::flight_param, this, &Strip::updateAltTargetDiff);

    auto update_selection = [=](QString ac) {
        bool selected = (ac == _ac_id);
        if(selected) {
            name_label->setStyleSheet("font-weight: bold; border: none; background: transparent;");
            name_label->setText(QString("<b>%1 (%2)</b>").arg(AircraftManager::get()->getAircraft(_ac_id)->name(), _ac_id));
        } else {
            name_label->setStyleSheet("font-weight: normal; border: none; background: transparent;");
            name_label->setText(QString("%1 (%2)").arg(AircraftManager::get()->getAircraft(_ac_id)->name(), _ac_id));
        }
        for (auto btn : this->findChildren<QPushButton*>()) {
            btn->setEnabled(selected);
        }
    };
    connect(DispatcherUi::get(), &DispatcherUi::ac_selected, this, update_selection);
    update_selection(DispatcherUi::get()->getSelectedAcId());

    strips[_ac_id] = this;
}

Strip::~Strip() {
    strips.remove(_ac_id);
}

Strip* Strip::getStrip(QString ac_id) {
    return strips.value(ac_id, nullptr);
}

void Strip::setCompact(bool compact) {
    full_strip->setVisible(!compact);
    short_strip->setVisible(compact);
}


void Strip::build_full_strip() {
    full_strip = new QWidget(this);
    auto layout_full = new QHBoxLayout(full_strip);
    layout_full->setContentsMargins(0, 0, 0, 0);
    layout_full->setSpacing(2);

    auto grid_main = new QGridLayout();
    grid_main->setSpacing(2);
    layout_full->addLayout(grid_main);

    // Row 0: [Flight Time] [Speed] [Throttle] [Block Name]
    full_flight_time_label = new QLabel("00:00:00", full_strip);
    full_flight_time_label->setAlignment(Qt::AlignCenter);
    full_flight_time_label->setStyleSheet("font-weight: bold; border: none;");
    full_flight_time_label->setToolTip("Flight Time. Click to reset");
    full_flight_time_label->installEventFilter(this);
    grid_main->addWidget(full_flight_time_label, 0, 0);

    full_speed_label = new JaugeLabel(0, 10, "m/s", full_strip);
    full_speed_label->setPrecision(1);
    full_speed_label->setToolTip("Ground speed");
    grid_main->addWidget(full_speed_label, 0, 1);

    full_throttle_label = new JaugeLabel(0, 100, "%", full_strip);
    full_throttle_label->setPrecision(0);
    full_throttle_label->setToolTip("Throttle");
    grid_main->addWidget(full_throttle_label, 0, 2);

    full_block_name_label = new QLabel("N/A", full_strip);
    full_block_name_label->setStyleSheet("font-weight: bold; border: none;");
    full_block_name_label->setToolTip("Current navigation block");
    full_block_name_label->setMinimumWidth(85);
    grid_main->addWidget(full_block_name_label, 0, 3);

    // Row 1 & 2, Col 1: Status (Mode/RC/GPS)
    auto status_layout = new QVBoxLayout();
    status_layout->setSpacing(1);
    
    auto status_text = new QLabel("<i>Status</i>", full_strip);
    status_text->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    status_layout->addWidget(status_text);
    
    full_ap_mode_label = new ColorLabel(0, full_strip);
    full_ap_mode_label->setToolTip("Navigation mode. Click to restore AUTO");
    full_ap_mode_label->installEventFilter(this);
    full_ap_mode_label->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    status_layout->addWidget(full_ap_mode_label);

    full_fbw_mode_label = new ColorLabel(0, full_strip);
    full_fbw_mode_label->setToolTip("Radio Command status");
    full_fbw_mode_label->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    status_layout->addWidget(full_fbw_mode_label);

    full_gps_mode_label = new ColorLabel(0, full_strip);
    full_gps_mode_label->setToolTip("GPS status");
    full_gps_mode_label->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    status_layout->addWidget(full_gps_mode_label);
    
    grid_main->addLayout(status_layout, 1, 1, 2, 1);

    // Row 1, Col 0: Battery
    auto bat_layout = new QVBoxLayout();
    bat_layout->setSpacing(0);
    auto bat_text = new QLabel("<i>Bat</i>", full_strip);
    bat_text->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    double min_bat = 9.0;
    double max_bat = 12.5;
    auto ac = AircraftManager::get()->getAircraft(_ac_id);
    if (ac && ac->getAirframe()) {
        auto cat_bat = ac->getAirframe()->getDefine("CATASTROPHIC_BAT_LEVEL", "BAT");
        if (cat_bat) min_bat = cat_bat.value().value.toDouble();
        auto m_bat = ac->getAirframe()->getDefine("MAX_BAT_LEVEL", "BAT");
        if (m_bat) max_bat = m_bat.value().value.toDouble();
    }

    full_bat_graph = new GraphLabel(min_bat, max_bat, full_strip);
    full_bat_graph->setUnit("");//No V, In original code doesn't show unit we also leave it out, not better but just more same behaviour to mimic original,anyhow set to volts for tooltip
    full_bat_graph->setToolTip("Battery level (V)");
    full_bat_graph->setMinimumSize(50, 50);
    
    // Allow the gauge to expand vertically and push label up
    full_bat_graph->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    
    bat_layout->addWidget(bat_text);
    bat_layout->addWidget(full_bat_graph);
    grid_main->addLayout(bat_layout, 1, 0);

    // Row 2, Col 0: Link
    auto link_layout = new QVBoxLayout();
    link_layout->setSpacing(0);
    auto link_text = new QLabel("<i>Link</i>", full_strip);
    link_text->setAlignment(Qt::AlignLeft);
    full_link_label = new ColorLabel(0, full_strip);
    full_link_label->setToolTip("Telemetry status");
    link_layout->addWidget(link_text);
    link_layout->addWidget(full_link_label);
    grid_main->addLayout(link_layout, 2, 0);

    // Row 1, Col 2: AGL
    auto agl_layout = new QVBoxLayout();
    agl_layout->setSpacing(0);
    auto agl_text = new QLabel("<i>AGL</i>", full_strip);
    agl_text->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    
    full_alt_graph = new GraphLabel(0, 150, full_strip);
    full_alt_graph->setUnit("m");
    full_alt_graph->setPrecision(0);
    full_alt_graph->setToolTip("AGL, climb indicator, vertical speed (m/s)");
    full_alt_graph->setIndicator(true);
    full_alt_graph->setDualText(true);
    full_alt_graph->setMinimumSize(50, 50);
    full_alt_graph->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    
    agl_layout->addWidget(agl_text);
    agl_layout->addWidget(full_alt_graph);
    grid_main->addLayout(agl_layout, 1, 2);

    // Row 2, Col 2: Target Alt Diff
    auto target_diff_layout = new QVBoxLayout();
    target_diff_layout->setSpacing(0);
    auto target_diff_text = new QLabel("<i>/Target</i>", full_strip);
    target_diff_text->setAlignment(Qt::AlignLeft);
    full_target_diff_label = new QLabel("-30m", full_strip);
    full_target_diff_label->setStyleSheet("font-weight: bold; border: none;");
    full_target_diff_label->setAlignment(Qt::AlignCenter);
    full_target_diff_label->setToolTip("Current altitude minus target altitude");
    target_diff_layout->addWidget(target_diff_text);
    target_diff_layout->addWidget(full_target_diff_label);
    grid_main->addLayout(target_diff_layout, 2, 2);

    // Row 2, Col 3: Altitude
    auto alt_value_layout = new QVBoxLayout();
    alt_value_layout->setSpacing(0);
    auto alt_text = new QLabel("<i>Alt</i>", full_strip);
    alt_text->setAlignment(Qt::AlignLeft);
    full_alt_value_label = new QLabel("185m / 215m", full_strip);
    full_alt_value_label->setStyleSheet("font-weight: bold; border: none;");
    full_alt_value_label->setAlignment(Qt::AlignCenter);
    full_alt_value_label->setToolTip("Current altitude / Target altitude");
    alt_value_layout->addWidget(alt_text);
    alt_value_layout->addWidget(full_alt_value_label);
    grid_main->addLayout(alt_value_layout, 2, 3);

    // Row 1, Col 3: Block Metrics Table
    auto block_layout = new QVBoxLayout();
    block_layout->setSpacing(0);
    auto block_text = new QLabel("<i>Block</i>", full_strip);
    block_text->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    block_layout->addWidget(block_text);

    auto grid_metrics = new QGridLayout();
    grid_metrics->setSpacing(1);
    grid_metrics->setContentsMargins(0, 0, 0, 0);

    auto add_metric = [&](QString label, QLabel*& val_lbl, int row, QString tooltip) -> QLabel* {
        auto l = new QLabel("<i>" + label + "</i>", full_strip);
        l->setToolTip(tooltip);
        grid_metrics->addWidget(l, row, 0);
        val_lbl = new QLabel("-", full_strip);
        val_lbl->setToolTip(tooltip);
        val_lbl->setStyleSheet("font-weight: bold;");
        grid_metrics->addWidget(val_lbl, row, 1);
        return l;
    };

    add_metric("Time", full_block_time_label, 0, "Block time");
    add_metric("Stage", full_stage_time_label, 1, "Stage time");
    add_metric("ETA", full_eta_time_label, 2, "Estimated Time of Arrival");
    full_apt_label = add_metric("Apt", full_apt_value_label, 3, "Appointment Time");
    full_apt_label->installEventFilter(this);
    
    btn_mark = new QPushButton("Mark", full_strip);
    btn_mark->setToolTip("Mark current position");
    grid_metrics->addWidget(btn_mark, 4, 1);

    block_layout->addLayout(grid_metrics);
    grid_main->addLayout(block_layout, 1, 3);

    // User extensions layout
    layout_user = new QHBoxLayout();
    layout_full->addLayout(layout_user);

    // Control Buttons Grid (Navigation)
    auto grid_btns = new QGridLayout();
    grid_btns->setSpacing(1);
    layout_full->addLayout(grid_btns);

    auto pprz_home = appConfig()->value("PAPARAZZI_HOME").toString();
    auto icon_path = pprz_home + "/data/pictures/gcs_icons/";

    btn_launch = new QPushButton(QIcon(icon_path + "launch.png"), "", full_strip);
    btn_launch->setToolTip("Launch");
    btn_kill = new QPushButton(QIcon(icon_path + "kill.png"), "", full_strip);
    btn_kill->setToolTip("Kill");
    btn_resurrect = new QPushButton(QIcon(icon_path + "resurrect.png"), "", full_strip);
    btn_resurrect->setToolTip("Resurrect");

    btn_up_up = new QPushButton(QIcon(icon_path + "upup.png"), "", full_strip);
    btn_up = new QPushButton(QIcon(icon_path + "up.png"), "", full_strip);
    btn_down = new QPushButton(QIcon(icon_path + "down.png"), "", full_strip);

    btn_left = new QPushButton(QIcon(icon_path + "left.png"), "", full_strip);
    btn_center = new QPushButton(QIcon(icon_path + "recenter.png"), "", full_strip);
    btn_right = new QPushButton(QIcon(icon_path + "right.png"), "", full_strip);

    grid_btns->addWidget(btn_launch, 0, 0);
    grid_btns->addWidget(btn_kill, 0, 1);
    grid_btns->addWidget(btn_resurrect, 0, 2);

    grid_btns->addWidget(btn_down, 1, 0);
    grid_btns->addWidget(btn_up, 1, 1);
    grid_btns->addWidget(btn_up_up, 1, 2);

    grid_btns->addWidget(btn_left, 2, 0);
    grid_btns->addWidget(btn_center, 2, 1);
    grid_btns->addWidget(btn_right, 2, 2);

    // Initial values and logic for buttons
    ac = AircraftManager::get()->getAircraft(_ac_id);

    connect(btn_launch, &QPushButton::clicked, this, [=]() {
        auto settings = ac->getSettingMenu()->getAllSettings();
        for(auto set : settings) {
            if(set->getName() == "autopilot.launch") {
                ac->setSetting(set, 1);
                break;
            }
        }
    });

    connect(btn_kill, &QPushButton::clicked, this, [=]() {
        bool confirm = true;
        if (appConfig()->contains("CONFIRM_KILL")) {
            confirm = appConfig()->value("CONFIRM_KILL").toBool();
        }
        QMessageBox::StandardButton reply = QMessageBox::Yes;
        if (confirm) {
            reply = QMessageBox::question(this, "Kill throttle", QString("Kill throttle of A/C %1 ?").arg(_ac_id), QMessageBox::Yes|QMessageBox::No);
        }
        if (reply == QMessageBox::Yes) {
            auto settings = ac->getSettingMenu()->getAllSettings();
            for(auto set : settings) {
                if(set->getName() == "autopilot.kill_throttle") {
                    ac->setSetting(set, 1);
                    break;
                }
            }
        }
    });

    connect(btn_resurrect, &QPushButton::clicked, this, [=]() {
        auto settings = ac->getSettingMenu()->getAllSettings();
        for(auto set : settings) {
            if(set->getName() == "autopilot.kill_throttle") {
                ac->setSetting(set, 0);
                break;
            }
        }
    });

    auto shift_alt = [=](float shift) {
        auto settings = ac->getSettingMenu()->getAllSettings();
        for(auto set : settings) {
            if(set->getName() == "altitude" || set->getName() == "flight_altitude") {
                float target_alt = 0;
                auto msg = ac->getStatus()->getMessage("NAV_STATUS");
                if(msg) msg->getField("target_alt", target_alt);
                ac->setSetting(set, target_alt + shift);
                break;
            }
        }
    };

    connect(btn_up, &QPushButton::clicked, this, [=]() { shift_alt(ac->getAirframe()->getAltShiftPlus()); });
    connect(btn_down, &QPushButton::clicked, this, [=]() { shift_alt(ac->getAirframe()->getAltShiftMinus()); });
    connect(btn_up_up, &QPushButton::clicked, this, [=]() { shift_alt(ac->getAirframe()->getAltShiftPlusPlus()); });

    auto lateral_shift = [=](float val) {
        auto settings = ac->getSettingMenu()->getAllSettings();
        for(auto set : settings) {
             if(set->getName() == "inc. shift") {
                 ac->setSetting(set, val);
                 break;
             }
        }
    };

    connect(btn_left, &QPushButton::clicked, this, [=]() { lateral_shift(-5.0); });
    connect(btn_center, &QPushButton::clicked, this, [=]() { lateral_shift(0.0); });
    connect(btn_right, &QPushButton::clicked, this, [=]() { lateral_shift(5.0); });

    // Refine Tooltips to match OCaml
    float altShiftPlus = ac->getAirframe()->getAltShiftPlus();
    float altShiftMinus = ac->getAirframe()->getAltShiftMinus();
    float altShiftPlusPlus = ac->getAirframe()->getAltShiftPlusPlus();

    btn_up->setToolTip(QString("Altitude +%1m").arg(altShiftPlus, 0, 'f', 1));
    btn_down->setToolTip(QString("Altitude %1m").arg(altShiftMinus, 0, 'f', 1));
    btn_up_up->setToolTip(QString("Altitude +%1m").arg(altShiftPlusPlus, 0, 'f', 1));

    btn_left->setToolTip("Shift 5m left ");
    btn_center->setToolTip("Re-center");
    btn_right->setToolTip("Shift 5m right ");

    connect(btn_mark, &QPushButton::clicked, this, [=]() {
        pprzlink::Message msg(PprzDispatcher::get()->getDict()->getDefinition("MARK"));
        msg.addField("ac_id", static_cast<uint8_t>(ac->getId().toUInt()));
        msg.addField("lat", static_cast<float>(ac->getPosition().lat()));
        msg.addField("long", static_cast<float>(ac->getPosition().lon()));
        PprzDispatcher::get()->sendMessage(msg);
    });

}

void Strip::addWidget(QWidget* w, QString group) {
    if(group == "") {
        layout_user->addWidget(w);
    } else {
        if(!user_group_layouts.contains(group)) {
            auto vLay = new QVBoxLayout();
            user_group_layouts[group] = vLay;
            layout_user->addLayout(vLay);
        }
        user_group_layouts[group]->addWidget(w);
    }
}

void Strip::build_short_strip() {
    short_strip = new QWidget(this);
    auto layout_short = new QGridLayout(short_strip);
    layout_short->setContentsMargins(2, 2, 2, 2);
    layout_short->setSpacing(1);

    double min_bat = 9.0;  // Default min battery voltage, can be overridden by airframe defines
    double max_bat = 12.5; // Default max battery voltage, can be overridden by airframe defines
    auto ac = AircraftManager::get()->getAircraft(_ac_id);
    if (ac && ac->getAirframe()) {
        auto cat_bat = ac->getAirframe()->getDefine("CATASTROPHIC_BAT_LEVEL", "BAT");
        if (cat_bat) min_bat = cat_bat.value().value.toDouble();
        auto m_bat = ac->getAirframe()->getDefine("MAX_BAT_LEVEL", "BAT");
        if (m_bat) max_bat = m_bat.value().value.toDouble();
    }

    // Row 0: Bat and Flight Time
    short_jl_bat = new JaugeLabel(min_bat, max_bat, "", short_strip);//Empty unit (V) since in original also doesn't show it, but set to volts for tooltip
    short_jl_bat->setToolTip("Battery level (V)");
    layout_short->addWidget(short_jl_bat, 0, 0);

    short_flight_time_label = new QLabel("00:00:00", short_strip);
    short_flight_time_label->setStyleSheet("font-weight: bold; border: none;");
    short_flight_time_label->setAlignment(Qt::AlignCenter);
    short_flight_time_label->setToolTip("Flight Time. Click to reset");
    short_flight_time_label->installEventFilter(this);
    layout_short->addWidget(short_flight_time_label, 0, 1);

    // Row 1: AP Mode,  RC Status and GPS Status
    short_ap_mode_label = new ColorLabel(0, short_strip);
    short_ap_mode_label->installEventFilter(this);
    layout_short->addWidget(short_ap_mode_label, 1, 0);

    short_fbw_mode_label = new ColorLabel(0, short_strip);
    layout_short->addWidget(short_fbw_mode_label, 1, 1);

    short_gps_mode_label = new ColorLabel(0, short_strip);
    layout_short->addWidget(short_gps_mode_label, 2, 0);

    // Row 2: Altitude and Vertical Speed
    short_alt_label = new QLabel("-m", short_strip);
    short_alt_label->setStyleSheet("font-weight: bold; border: 1px solid gray;");
    short_alt_label->setAlignment(Qt::AlignCenter);
    layout_short->addWidget(short_alt_label, 2, 1);

    // Row 3: Speed and Vertical Speed
    short_speed_label = new QLabel("-m/s", short_strip);
    short_speed_label->setStyleSheet("font-weight: bold; border: 1px solid gray;");
    short_speed_label->setAlignment(Qt::AlignCenter);
    layout_short->addWidget(short_speed_label, 3, 0);

    auto vs_layout = new QHBoxLayout();
    short_vspeed_indicator = new QLabel(QString::fromUtf8("\xE2\x86\x92"), short_strip);
    short_vspeed_label = new QLabel("+-", short_strip);
    vs_layout->addWidget(short_vspeed_indicator);
    vs_layout->addWidget(short_vspeed_label);
    layout_short->addLayout(vs_layout, 3, 1);

    // Row 4: Target Alt Diff (mini doesn't have it in glade but useful here)
    short_target_label = new QLabel("-m", short_strip);
    short_target_label->setStyleSheet("color: gray; font-size: 10px;");
    short_target_label->setAlignment(Qt::AlignCenter);
    layout_short->addWidget(short_target_label, 4, 0, 1, 2);
}


void Strip::paintEvent(QPaintEvent* e) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    
    // Smooth background with subtle gradient
    QLinearGradient gradient(rect().topLeft(), rect().bottomLeft());
    gradient.setColorAt(0, ac_color);
    gradient.setColorAt(1, ac_color.darker(140));
    
    QPainterPath path;
    path.addRoundedRect(rect().adjusted(1, 1, -1, -1), 4, 4);
    p.fillPath(path, gradient);
    
    // Subtle border
    p.setPen(QPen(ac_color.darker(155), 1));
    p.drawPath(path);

    QWidget::paintEvent(e);
}

void Strip::mousePressEvent(QMouseEvent *e) {
    (void)e;
    emit DispatcherUi::get()->ac_selected(_ac_id);
}

void Strip::mouseReleaseEvent(QMouseEvent *e) {
    (void)e;
}

void Strip::updateEngineStatus() {
    auto msg = AircraftManager::get()->getAircraft(_ac_id)->getStatus()->getMessage("ENGINE_STATUS");
    if(msg) {
        float bat, throttle;
        msg->getField("throttle", throttle);
        msg->getField("bat", bat);
        full_bat_graph->pushData(bat);
        short_jl_bat->setValue(bat);
        full_throttle_label->setValue(throttle);
    }
}

void Strip::updateApStatus() {
    auto msg = AircraftManager::get()->getAircraft(_ac_id)->getStatus()->getMessage("AP_STATUS");
    if(msg) {
        QString ap_mode, lat_mode, horiz_mode, gaz_mode, gps_mode, kill_mode;
        uint32_t flight_time;

        msg->getField("flight_time", flight_time);
        msg->getField("kill_mode", kill_mode);
        msg->getField("ap_mode", ap_mode);
        msg->getField("gps_mode", gps_mode);

        int hours = static_cast<int>(static_cast<int64_t>(flight_time)/3600);//3600 is an hour, use int64_t to prevent overflow for long flight times
        int minutes = static_cast<int>(static_cast<int64_t>(flight_time)/60 - hours*60);
        int seconds = static_cast<int>(static_cast<int64_t>(flight_time) - minutes*60 -hours*3600);

        QString f_time = QString("%1").arg(hours, 2, 10, QChar('0')) + ":" +
                         QString("%1").arg(minutes, 2, 10, QChar('0')) + ":" +
                         QString("%1").arg(seconds, 2, 10, QChar('0'));
        full_flight_time_label->setText(f_time);
        short_flight_time_label->setText(f_time);

        full_throttle_label->setStatus(kill_mode=="OFF");

        if(ap_mode == "HOME" || ap_mode == "FAILSAFE") {
            full_ap_mode_label->setBrush(Qt::red);
            short_ap_mode_label->setBrush(Qt::red);
        } else if (ap_mode == "MANUAL") {
            full_ap_mode_label->setBrush(QColor(0xffa500));
            short_ap_mode_label->setBrush(QColor(0xffa500));
        } else if (ap_mode == "AUTO2") {
            full_ap_mode_label->setBrush(QColor(0x00ff00));
            short_ap_mode_label->setBrush(QColor(0x00ff00));
        } else {
            full_ap_mode_label->setBrush(QColor(0x10f0e0)); // Light blue
            short_ap_mode_label->setBrush(QColor(0x10f0e0));
        }
        full_ap_mode_label->setText(ap_mode);
        short_ap_mode_label->setText(ap_mode);


        if(gps_mode == "NOFIX") {
            full_gps_mode_label->setBrush(Qt::red);
            short_gps_mode_label->setBrush(Qt::red);
        } else if (gps_mode == "NA" || gps_mode == "2D") {
            full_gps_mode_label->setBrush(QColor(0xffa500));
            short_gps_mode_label->setBrush(QColor(0xffa500));
        } else {
            full_gps_mode_label->setBrush(QColor(0x00ff00));
            short_gps_mode_label->setBrush(QColor(0x00ff00));
        }
        full_gps_mode_label->setText(gps_mode);
        short_gps_mode_label->setText(gps_mode);

    }
}

void Strip::updateAltTargetDiff() {
    auto nav_status_msg = AircraftManager::get()->getAircraft(_ac_id)->getStatus()->getMessage("NAV_STATUS");
    auto flight_param_msg = AircraftManager::get()->getAircraft(_ac_id)->getStatus()->getMessage("FLIGHT_PARAM");
    if(nav_status_msg && flight_param_msg) {
        float target_alt;
        nav_status_msg->getField("target_alt", target_alt);

        float alt;
        flight_param_msg->getField("alt", alt);

        auto diff = alt - target_alt;
        auto display_diff = Units::get()->convert(static_cast<double>(diff), "m", "m");
        full_target_diff_label->setText(QString::number(display_diff, 'f', 1) + "m");
        full_alt_value_label->setText(QString::number(alt, 'f', 0) + "m / " + QString::number(target_alt, 'f', 0) + "m");
        short_target_label->setText(QString::number(display_diff, 'f', 1) + " m");

        // Update Block/Stage info
        uint8_t cur_block, cur_stage;
        uint32_t block_time, stage_time;
        nav_status_msg->getField("cur_block", cur_block);
        nav_status_msg->getField("cur_stage", cur_stage);
        nav_status_msg->getField("block_time", block_time);
        nav_status_msg->getField("stage_time", stage_time);

        auto ac = AircraftManager::get()->getAircraft(_ac_id);
        auto fp = ac->getFlightPlan();
        if(fp) {
            auto block = fp->getBlock(cur_block);
            if(block) {
                full_block_name_label->setText(block->getName());
            }
        }

        // removed full_stage_name_label text set as stage name is not sent by nav_status anymore?
        auto format_time = [](uint32_t t) {
            int m = t / 60;
            int s = t % 60;
            return QString("%1:%2").arg(m, 2, 10, QChar('0')).arg(s, 2, 10, QChar('0'));
        };

        full_block_time_label->setText(format_time(block_time));
        full_stage_time_label->setText(format_time(stage_time));

        float desired_tow = -1.0;
        try {
            nav_status_msg->getField("snav_desired_tow", desired_tow);
        } catch (...) {}
        if (desired_tow >= 0) {
             int rdv = static_cast<int>(desired_tow);
             int h = (rdv / 3600) % 24;
             int m = (rdv / 60) % 60;
             int s = rdv % 60;
             full_apt_value_label->setText(QString("%1:%2:%3").arg(h, 2, 10, QChar('0')).arg(m, 2, 10, QChar('0')).arg(s, 2, 10, QChar('0')));
        }
    }
}

void Strip::updateFlightParams() {
    auto msg = AircraftManager::get()->getAircraft(_ac_id)->getStatus()->getMessage("FLIGHT_PARAM");
    if(msg) {
        float speed, alt, climb, agl, airspeed;
        msg->getField("speed", speed);
        msg->getField("alt", alt);
        msg->getField("climb", climb);
        msg->getField("agl", agl);
        msg->getField("airspeed", airspeed);
        full_speed_label->setValue(Units::get()->convert(speed, "m/s", "m/s"));
        full_speed_label->setToolTip(QString("Ground speed (est. airspeed: %1m/s)").arg(Units::get()->convert(airspeed, "m/s", "m/s"), 0, 'f', 1));
        short_speed_label->setText(QString::number(Units::get()->convert(speed, "m/s", "m/s"), 'f', 1) + " m/s");
        
        full_alt_graph->pushData(Units::get()->convert(agl, "m", "m"));
        full_alt_graph->setToolTip(QString("AGL: %1m, climb: %2m/s").arg(agl, 0, 'f', 0).arg(climb, 0, 'f', 1));
        
        short_alt_label->setText(QString::number(Units::get()->convert(agl, "m", "m"), 'f', 0) + " m");
        QString txt = QString::number(climb, 'f', 1);
        if(climb > 0) {
            txt = "+" + txt;
        }
        full_alt_graph->setSecondayText(txt);
        short_vspeed_label->setText(txt);
        if(abs(speed) > 0.1) {
            full_alt_graph->setIndicatorAngle((climb/speed)*1.2);
        } else if(abs(climb) > 0.01){
            full_alt_graph->setIndicatorAngle(climb/abs(climb)*0.3);
        } else {
            full_alt_graph->setIndicatorAngle(0);
        }

        if(climb > 0.5) {
            short_vspeed_indicator->setText(QString::fromUtf8("\xE2\x86\x97"));
        } else if(climb < -0.5) {
            short_vspeed_indicator->setText(QString::fromUtf8("\xE2\x86\x98"));
        } else {
            short_vspeed_indicator->setText(QString::fromUtf8("\xE2\x86\x92"));
        }

        // Update ETA if possible (requires more navigation info, for now just update alt diff)
        updateAltTargetDiff();
    }
}


bool Strip::eventFilter(QObject *watched, QEvent *event) {
    if (event->type() == QEvent::MouseButtonPress) {
        if (watched == full_flight_time_label || watched == short_flight_time_label) {
            auto res = QMessageBox::question(this, "Reset flight time", "Reset flight time for A/C " + _ac_id + " ?");
            if (res == QMessageBox::Yes) {
                auto ac = AircraftManager::get()->getAircraft(_ac_id);
                auto settings = ac->getSettingMenu()->getAllSettings();
                for(auto set : settings) {
                    if(set->getFullName() == "autopilot.flight_time") {
                        ac->setSetting(set, 0.0f);
                        break;
                    }
                }
                qDebug() << "Resetting flight time for " << _ac_id;
            }
            return true;
        } else if (watched == full_ap_mode_label || watched == short_ap_mode_label) {
             auto res = QMessageBox::question(this, "Back to auto", "Restore AUTO mode for A/C " + _ac_id + " ?");
             if (res == QMessageBox::Yes) {
                 auto ac = AircraftManager::get()->getAircraft(_ac_id);
                 auto settings = ac->getSettingMenu()->getAllSettings();
                 for(auto set : settings) {
                     if(set->getFullName() == "autopilot.mode") {
                         ac->setSetting(set, 2.0f); // 2 is typically AUTO2
                         break;
                     }
                 }
             }
             return true;
        } else if (watched == full_apt_label) {
            QDialog dialog(this);
            dialog.setWindowTitle("Set Appointment Time");
            auto layout = new QVBoxLayout(&dialog);
            auto timeEdit = new QTimeEdit(QTime::currentTime().addSecs(60), &dialog);
            timeEdit->setDisplayFormat("HH:mm:ss");
            layout->addWidget(new QLabel("Enter Appointment Time (UTC):", &dialog));
            layout->addWidget(timeEdit);
            auto buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
            layout->addWidget(buttonBox);
            connect(buttonBox, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
            connect(buttonBox, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);

            if (dialog.exec() == QDialog::Accepted) {
                QTime time = timeEdit->time();
                // GPS TOW: ((wday*24 + hour)*60+min)*60+sec + leap_seconds
                // For simplicity, we use the current wday.
                int wday = QDate::currentDate().dayOfWeek() % 7; // Sunday=0 in Paparazzi? 
                // Paparazzi OCaml tm_wday: 0 is Sunday. Qt dayOfWeek: 1 is Monday, 7 is Sunday.
                if (wday == 7) wday = 0; 
                
                int leap_seconds = 16; // Matching original latlong.ml
                double tow = ((wday * 24 + time.hour()) * 60 + time.minute()) * 60 + time.second() + leap_seconds;
                
                auto ac = AircraftManager::get()->getAircraft(_ac_id);
                auto settings = ac->getSettingMenu()->getAllSettings();
                for(auto set : settings) {
                    if(set->getName() == "snav_desired_tow") {
                        ac->setSetting(set, static_cast<float>(tow));
                        break;
                    }
                }
            }
            return true;
        }
    }
    return QWidget::eventFilter(watched, event);
}

void Strip::updateTelemetryStatus() {
    auto msg = AircraftManager::get()->getAircraft(_ac_id)->getStatus()->getTelemetryMessages().isEmpty() ? std::nullopt : std::make_optional(AircraftManager::get()->getAircraft(_ac_id)->getStatus()->getTelemetryMessages().first());
    if(msg) {
        float time_since_last_msg;
        msg->getField("time_since_last_msg", time_since_last_msg);
        if(time_since_last_msg > 5) {
            full_link_label->setBrush(Qt::red);
        } else {
            full_link_label->setBrush(QColor(0x00ff00));
        }

        if(time_since_last_msg > 2) {
            full_link_label->setText(QString::number(time_since_last_msg, 'f', 0));
        } else {
            full_link_label->setText("");
        }
    }
}

void Strip::updateFBW() {
    auto msg = AircraftManager::get()->getAircraft(_ac_id)->getStatus()->getMessage("FLY_BY_WIRE");
    if(msg) {
        QString rc_status, rc_mode;
        msg->getField("rc_status", rc_status);
        msg->getField("rc_mode", rc_mode);

        if(rc_status == "OK") {
            full_fbw_mode_label->setBrush(QColor(0x00ff00));
            short_fbw_mode_label->setBrush(QColor(0x00ff00));
        } else if (rc_status == "LOST" || rc_status == "REALLY_LOST") {
            full_fbw_mode_label->setBrush(Qt::red);
            short_fbw_mode_label->setBrush(Qt::red);
        } else {
            full_fbw_mode_label->setBrush(QColor(0xffa500));
            short_fbw_mode_label->setBrush(QColor(0xffa500));
        }

        full_fbw_mode_label->setText(rc_status);
        short_fbw_mode_label->setText(rc_status);

    }
}


/* --- HISTORICAL OCAML COMMENTS: strip.ml ---
 * * set a label
 ------------------------------------------
 * * set a color
 ------------------------------------------
 * since tcl8.6 "green" refers to "darkgreen" and the former "green" is now "lime", but that is not available in older versions, so hardcode the color to #00ff00
 ------------------------------------------
 * Else the drawing area is not allocated already
 ------------------------------------------
 * First call: fill the array with the given value
 ------------------------------------------
 * Store the value in the history array and update index
 ------------------------------------------
 * From left to right, older to new values
 ------------------------------------------
 * Arrow for the variation
 ------------------------------------------
 * Else the drawing area is not allocated already
 ------------------------------------------
 * * add a strip to the panel
 ------------------------------------------
 * let add = fun config color min_bat max_bat ->
 ------------------------------------------
 * Name in top left
 ------------------------------------------
 * battery gauge
 ------------------------------------------
 * AGL gauge
 ------------------------------------------
 * Speed gauge
 ------------------------------------------
 * Throttle gauge
 ------------------------------------------
 * Diff to target altitude
 ------------------------------------------
 * Telemetry
 ------------------------------------------
 * Labels
 ------------------------------------------
 * Buttons : setting the icons (the path of the icon is not saved by glade)
 ------------------------------------------
 * add a button widget in a vertical box if it belongs to a group (create new group if needed)
 ------------------------------------------
 * let vbox = GPack.vbox ~show:true () in
 ------------------------------------------
 * No confirmation for resurrect or confirm_kill = false
 ------------------------------------------
 * Back in AUTO2
 ------------------------------------------
 * Reset the flight time
 ------------------------------------------
 * Reset flight time
 ------------------------------------------
 * * Appointment date
 ------------------------------------------
*/
