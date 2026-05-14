#include "app_settings.h"
#include <QApplication>
#include <QFile>
#include <QTextStream>
#include <QDebug>
#include <QFileInfo>
#include <QString>
#include <iostream>
#include <QDir>
#include <QVariant>
#include <QStandardPaths>
#include <QSettings>
#include <QtWidgets>
#include <QFileInfo>
#include <pprzmain.h>
#include "gcs_utils.h"
#include "globalconfig.h"

void default_setting(QString key, QVariant value) {
    auto settings = getAppSettings();

    if(!settings.contains(key)) {
        settings.setValue(key, value);
    }
}

void set_app_settings() {
    auto settings = getAppSettings();

    default_setting("ivy/name", "PprzGCS");
    default_setting("pprzlink/id", "pprzcontrol");

    if(!appConfig()->contains("IVY_BUS")) {
        appConfig()->setValue("IVY_BUS", "127.255.255.255:2010");
    }

    auto PAPARAZZI_HOME = QString(qgetenv("PAPARAZZI_HOME"));
    auto PAPARAZZI_SRC = QString(qgetenv("PAPARAZZI_SRC"));
    if(PAPARAZZI_HOME == "") {
        // Fallback robustly by checking where the executable is located rather than the unreliable working directory
        QString appDir = QCoreApplication::applicationDirPath();
        QDir d(appDir);
        while (!d.isRoot()) {
            if (QFile::exists(d.filePath("conf/flight_plans/flight_plan.dtd"))) {
                PAPARAZZI_HOME = d.absolutePath();
                break;
            }
            d.cdUp();
        }
        
        if (PAPARAZZI_HOME == "") {
            PAPARAZZI_HOME = QDir::currentPath() + "/../../.."; // default fallback relative to cockpit_qt/
        }
    }
    if(PAPARAZZI_SRC == "") {
        PAPARAZZI_SRC = PAPARAZZI_HOME;
    }

    appConfig()->setValue("PAPARAZZI_HOME", PAPARAZZI_HOME);
    appConfig()->setValue("PAPARAZZI_SRC", PAPARAZZI_SRC);

    if(!appConfig()->contains("MESSAGES")) {
        auto messages = appConfig()->value("PAPARAZZI_HOME").toString() + "/var/messages.xml";
        appConfig()->setValue("MESSAGES", messages);
    }

    if(!appConfig()->contains("GCS_ICONS_PATH")) {
        auto gcs_icons_path = appConfig()->value("PAPARAZZI_HOME").toString() + "/data/pictures/gcs_icons";
        appConfig()->setValue("GCS_ICONS_PATH", gcs_icons_path);
    }

    QString map_source = appConfig()->value("MAPS_SOURCE").toString();
    bool auto_ortho = appConfig()->value("AUTO_ORTHO").toBool();
    if (map_source == "osm") {
        default_setting("map/default_tiles", "OSM");
    } else if (map_source == "ms") {
        default_setting("map/default_tiles", auto_ortho ? "MS Hybrid" : "MS");
    } else {
        default_setting("map/default_tiles", auto_ortho ? "Google Hybrid" : "Google");
    }

    default_setting("map/move_hyteresis", 20);
    default_setting("map/waypoint/size", 8);
    default_setting("map/gcs_icon_size", 16);
    default_setting("map/gcs_icon_color", "#ffff00");
    default_setting("map/circle/stroke", 4);
    default_setting("map/size_highlight_factor", 1.5);

    default_setting("map/z_values/shapes", 50);
    default_setting("map/z_values/unhighlighted", 100);
    default_setting("map/z_values/highlighted", 200);
    default_setting("map/z_values/aircraft", 400);
    default_setting("map/z_values/carrot", 500);
    default_setting("map/z_values/nav_shape", 600);

    default_setting("map/items_font", 18);
    default_setting("map/aircraft/size", 40);
    default_setting("map/intruder_size", 40);

    default_setting("map/tiles_path", appConfig()->value("USER_DATA_PATH").toString() + "/map");

    int track_size = appConfig()->value("TRACK_SIZE", 500).toInt();
    default_setting("map/aircraft/track_size", track_size);
    default_setting("map/aircraft/track_width", 1);

    default_setting("aircraft_default_color", "red");

    default_setting("APP_STYLE_FILE", "default_style.qss");
    default_setting("APP_LAYOUT_FILE", appConfig()->value("LAYOUT_FILE").toString());

}

SettingsEditor::SettingsEditor(bool standalone, QWidget* parent): QDialog(parent)
{
    setWindowTitle("Settings");
    auto lay = new QVBoxLayout(this);

    auto tabWidget = new QTabWidget(this);
    lay->addWidget(tabWidget);

    auto settings = getAppSettings();

    // Tab Ivy
    auto w_ivy = new QWidget(tabWidget);
    auto l_ivy = new QGridLayout(w_ivy);
    int row = 0;
    auto cb = addSetting("Ivy name", "ivy/name", w_ivy, l_ivy, row, Type::STRING);
    callbacks.append(cb);
    cb = addSetting("Pprzlink ID", "pprzlink/id", w_ivy, l_ivy, row, Type::STRING);
    callbacks.append(cb);
    tabWidget->addTab(w_ivy, "Ivy");

    // Tab Map
    auto w_map = new QWidget(tabWidget);
    auto l_map = new QGridLayout(w_map);
    row = 0;

    QVariant tpnv = GlobalConfig::get()->value("tile_providers_names");
    if(tpnv.isValid()) {
        cb = addSetting("Default tiles", "map/default_tiles", w_map, l_map, row, Type::STRING_COMBO, tpnv.toStringList());
        callbacks.append(cb);
    } else {
        cb = addSetting("Default tiles", "map/default_tiles", w_map, l_map, row, Type::STRING);
        callbacks.append(cb);
    }
    cb = addSetting("Waypoint Size", "map/waypoint/size", w_map, l_map, row, Type::INT);
    callbacks.append(cb);
    cb = addSetting("Items Font", "map/items_font", w_map, l_map, row, Type::INT);
    callbacks.append(cb);
    cb = addSetting("Tiles path", "map/tiles_path", w_map, l_map, row, Type::PATH_DIR);
    callbacks.append(cb);
    cb = addSetting("Move Hysteresis", "map/move_hyteresis", w_map, l_map, row, Type::INT);
    callbacks.append(cb);
    cb = addSetting("Size highlight factor", "map/size_highlight_factor", w_map, l_map, row, Type::DOUBLE);
    callbacks.append(cb);
    cb = addSetting("Aircraft Size", "map/aircraft/size", w_map, l_map, row, Type::INT);
    callbacks.append(cb);
    cb = addSetting("Track size", "map/aircraft/track_size", w_map, l_map, row, Type::INT);
    callbacks.append(cb);
    cb = addSetting("Track width", "map/aircraft/track_width", w_map, l_map, row, Type::INT);
    callbacks.append(cb);
    tabWidget->addTab(w_map, "Map");

    // Tab General
    auto w_general = new QWidget(tabWidget);
    auto l_general = new QGridLayout(w_general);
    row = 0;

    auto user_layout_files = getLayoutFiles("USER_DATA_PATH");
    auto app_layout_files = getLayoutFiles("APP_DATA_PATH");
    cb = addSetting("Layout", "APP_LAYOUT_FILE", w_general, l_general, row, Type::STRING_COMBO, user_layout_files + app_layout_files);
    callbacks.append(cb);
    cb = addSetting("Style", "APP_STYLE_FILE", w_general, l_general, row, Type::PATH_FILE);
    callbacks.append(cb);
    tabWidget->addTab(w_general, "General");


    auto buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    lay->addWidget(buttonBox);

    connect(buttonBox->button(QDialogButtonBox::Ok), &QPushButton::clicked, this, [=](){
        for(auto &cb:callbacks) {
            cb();
        }

        auto res = QMessageBox::question(nullptr, "Restart application?", "Application need to be restarted to update settings. Do you want to restart now?");
        accept();
        if(res == QMessageBox::Yes) {
            qDebug() << "Restarting application...";
            PprzMain::launch_type = NORMAL;
            qApp->exit( PprzMain::EXIT_CODE_REBOOT );
        }
    });

    connect(buttonBox->button(QDialogButtonBox::Cancel), &QPushButton::clicked, this, [=](){
        reject();
        if(standalone) {
            qDebug() << "Restarting application...";
            PprzMain::launch_type = NORMAL;
            qApp->exit( PprzMain::EXIT_CODE_REBOOT );
        }
    });
}


std::function<void()> SettingsEditor::addSetting(QString name, QString key, QWidget* w, QGridLayout* gl, int &row, Type type, QVariant data) {
    auto settings = getAppSettings();
    auto label = new QLabel(name, w);

    int r = row;
    row += 1;


    gl->addWidget(label, r, 0);

    if(type == Type::STRING) {
        auto edit = new QLineEdit(settings.value(key).toString(), w);
        gl->addWidget(edit, r, 1);

        auto cb = [=]() {
          auto settings = getAppSettings();
          settings.setValue(key, edit->text());
        };

        return cb;
    } else if(type == Type::INT) {
        auto edit = new QLineEdit(settings.value(key).toString(), w);
        gl->addWidget(edit, r, 1);

        auto cb = [=]() {
          auto settings = getAppSettings();
          settings.setValue(key, edit->text().toInt());
        };

        return cb;
    } else if(type == Type::DOUBLE) {
        auto edit = new QLineEdit(settings.value(key).toString(), w);
        gl->addWidget(edit, r, 1);

        auto cb = [=]() {
          auto settings = getAppSettings();
          settings.setValue(key, edit->text().toDouble());
        };

        return cb;
    } else if(type == Type::PATH_DIR) {
        auto edit = new QLineEdit(settings.value(key).toString(), w);
        gl->addWidget(edit, r, 1);

        QToolButton* but = new QToolButton(w);
        but->setIcon(style()->standardIcon(QStyle::SP_DirOpenIcon));
        gl->addWidget(but, r, 2);
        connect(but, &QToolButton::clicked, this, [=]() {
            auto dir = QFileDialog::getExistingDirectory(w, name, edit->text());
            qDebug() << dir;
            if(dir != "") {
                edit->setText(dir);
            }
        });

        auto cb = [=]() {
          auto settings = getAppSettings();
          settings.setValue(key, edit->text());
        };

        return cb;
    }  else if(type == Type::PATH_FILE) {
        auto edit = new QLineEdit(settings.value(key).toString(), w);
        gl->addWidget(edit, r, 1);

        QToolButton* but = new QToolButton(w);
        but->setIcon(style()->standardIcon(QStyle::SP_DirOpenIcon));
        gl->addWidget(but, r, 2);
        connect(but, &QToolButton::clicked, this, [=]() {
            QFileInfo fi(edit->text());
            auto dir = QFileDialog::getOpenFileName(w, name, fi.absoluteDir().path());
            qDebug() << dir;
            if(dir != "") {
                edit->setText(dir);
            }
        });

        auto cb = [=]() {
          auto settings = getAppSettings();
          settings.setValue(key, edit->text());
        };

        return cb;
    } else if(type == Type::STRING_COMBO) {
        auto combo = new QComboBox(w);
        combo->addItems(data.toStringList());
        auto current = settings.value(key).toString();
        combo->setCurrentText(current);
        gl->addWidget(combo, r, 1);

        auto cb = [=]() {
            auto settings = getAppSettings();
            settings.setValue(key, combo->currentText());
        };

        return cb;
    }
    else {
        throw std::runtime_error("Error: Setting type not handled !");
    }
}
