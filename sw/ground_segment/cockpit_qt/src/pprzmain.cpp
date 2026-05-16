#include "pprzmain.h"
#include <QMessageBox>
#include <QInputDialog>
#include <QDebug>
#include "dispatcher_ui.h"
#include "AircraftManager.h"
#include "pprz_dispatcher.h"
#include <QSettings>
#include "app_settings.h"
#include "gcs_utils.h"

#include "speaker.h"
#include "globalconfig.h"

LaunchTypes PprzMain::launch_type = DEFAULT;

PprzMain::PprzMain(QWidget *parent) :
    QMainWindow(parent)
{

}

void PprzMain::setupUi(int width, int height, QWidget* centralWidget) {
    centralWidget->setParent(this);
    resize(width, height);
    menu_bar = new QMenuBar(this);
    menu_bar->setObjectName(QString::fromUtf8("menuBar"));
    menu_bar->setGeometry(QRect(0, 0, 555, 22));
    setMenuBar(menu_bar);
    mainToolBar = new QToolBar(this);
    mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
    addToolBar(Qt::TopToolBarArea, mainToolBar);
    statusBar = new QStatusBar(this);
    statusBar->setObjectName(QString::fromUtf8("statusBar"));
    setStatusBar(statusBar);
    setCentralWidget(centralWidget);
    setWindowIcon(QIcon(":/pictures/equinoxgcs.svg"));
    populate_menu();

    statusBar->addPermanentWidget(new QLabel("server status:"));

    serverStatusLed = new QLabel(statusBar);
    setServerStatus(false);
    statusBar->addPermanentWidget(serverStatusLed);
    statusBar->hide();

    connect(DispatcherUi::get(), &DispatcherUi::new_ac_config, this, &PprzMain::newAC);
    connect(DispatcherUi::get(), &DispatcherUi::ac_deleted, this, &PprzMain::removeAC);

    emit ready();
}

void PprzMain::setServerStatus(bool active) {
    QIcon ic;
    if(active) {
        ic = QIcon(":/pictures/green_led.svg");
    } else {
        ic = QIcon(":/pictures/red_led.svg");
    }
    serverStatusLed->setPixmap(ic.pixmap(15, 15));
}

void PprzMain::populate_menu() {
    // Nav Menu (legacy file_menu)
    auto nav_menu = menu_bar->addMenu("Nav");

    nav_menu->addAction("display SRTM alt");
    nav_menu->addAction("UTM Grid");
    nav_menu->addAction("UTC Time");
    nav_menu->addAction("Background");
    nav_menu->addAction("Goto");
    
    auto fit_to_window = nav_menu->addAction("Fit to window (f)");
    fit_to_window->setShortcut(QKeySequence("F"));
    connect(fit_to_window, &QAction::triggered, [=](){ emit DispatcherUi::get()->fitToWindow(); });

    auto auto_hide_fp = nav_menu->addAction("Auto hide FP");
    auto_hide_fp->setCheckable(true);
    auto_hide_fp->setChecked(appConfig()->value("AUTO_HIDE_FP", false).toBool());
    connect(auto_hide_fp, &QAction::toggled, [=](bool checked){ appConfig()->setValue("AUTO_HIDE_FP", checked); });

    auto redraw = nav_menu->addAction("Redraw");
    redraw->setShortcut(QKeySequence("L"));
    connect(redraw, &QAction::triggered, [=](){
        // Stub for redraw
        qDebug() << "Redraw maps";
    });

    auto fullscreen = nav_menu->addAction(QIcon::fromTheme("view-fullscreen"), "Fullscreen");
    fullscreen->setShortcut(QKeySequence("F11"));
    connect(fullscreen, &QAction::triggered, [=](){
        if(this->isFullScreen()) { this->showNormal(); }
        else { this->showFullScreen(); }
    });

    auto save_layout = nav_menu->addAction("Save layout");
    save_layout->setShortcut(QKeySequence("S"));
    connect(save_layout, &QAction::triggered, [=](){ qDebug() << "Save layout"; });

    auto restart_plugin = nav_menu->addAction("Restart plugin");
    restart_plugin->setShortcut(QKeySequence("P"));
    connect(restart_plugin, &QAction::triggered, [=](){ qDebug() << "Restart plugin"; });
    restart_plugin->setVisible(false);

    auto swap_plugin_map = nav_menu->addAction("Swap plugin/map");
    connect(swap_plugin_map, &QAction::triggered, [=](){ qDebug() << "Swap plugin/map"; });
    swap_plugin_map->setVisible(false);

    // Legacy C++ specific settings ported over:
    auto edit_settings = nav_menu->addAction("Edit Settings");
    connect(edit_settings, &QAction::triggered, [=](){
        auto se = new SettingsEditor();
        se->open();
    });
    auto speech_action = nav_menu->addAction("Enable Speech");
    speech_action->setCheckable(true);
    bool enable_speech = GlobalConfig::get()->value("SPEECH_ENABLED").toBool();
    setSpeech(enable_speech);
    
    // Set checked state but prevent the signal from firing since we manually initialize speaker
    speech_action->blockSignals(true);
    speech_action->setChecked(enable_speech);
    speech_action->blockSignals(false);
    
    // Explicitly seed the initial speech state to speaker tool 
    if(enable_speech && pprzApp() && pprzApp()->toolbox() && pprzApp()->toolbox()->speaker()) {
        pprzApp()->toolbox()->speaker()->enableSpeech(true, true);
    }
    
    connect(speech_action, &QAction::toggled, pprzApp()->toolbox()->speaker(), [=](bool s) {
        pprzApp()->toolbox()->speaker()->enableSpeech(s, false);
    });

    nav_menu->addSeparator();

    auto quit = nav_menu->addAction("Quit");
    quit->setShortcut(QKeySequence("Q"));
    connect(quit, &QAction::triggered, this, &QWidget::close);

    // Maps Menu
    auto map_menu = menu_bar->addMenu("Maps");

    auto load_user_map = map_menu->addAction("Load User Map");
    load_user_map->setShortcut(QKeySequence("M"));

    auto calibrate = map_menu->addAction("Calibrate");
    calibrate->setShortcut(QKeySequence("Shift+C"));

    auto maps_source_menu = map_menu->addMenu("Maps Source");
    QActionGroup* sourceGroup = new QActionGroup(this);
    QStringList sources = {"Google", "OSM", "MS", "IGN"};
    for(const QString& src : sources) {
        auto act = maps_source_menu->addAction(src);
        act->setCheckable(true);
        sourceGroup->addAction(act);
    }
    sourceGroup->actions().first()->setChecked(true);

    auto maps_policy_menu = map_menu->addMenu("Maps Policy");
    QActionGroup* policyGroup = new QActionGroup(this);
    QStringList policies = {"Cache Or Download", "Cache Then Download", "Local Only"};
    for(const QString& src : policies) {
        auto act = maps_policy_menu->addAction(src);
        act->setCheckable(true);
        policyGroup->addAction(act);
    }
    policyGroup->actions().first()->setChecked(true);

    auto maps_fill = map_menu->addAction("Maps Fill");
    maps_fill->setShortcut(QKeySequence("G"));

    auto maps_auto = map_menu->addAction("Maps Auto");
    maps_auto->setCheckable(true);
    
    auto map_of_region = map_menu->addAction("Map of Region");
    map_of_region->setShortcut(QKeySequence("R"));

    auto dump_map_of_tiles = map_menu->addAction("Dump map of Tiles");
    dump_map_of_tiles->setShortcut(QKeySequence("T"));

    auto load_sector = map_menu->addAction("Load sector");
    connect(load_sector, &QAction::triggered, [=](){
        QString fileName = QFileDialog::getOpenFileName(this, "Load sectors", appConfig()->value("FLIGHT_PLANS_PATH").toString(), "XML files (*.xml)");
        if(!fileName.isEmpty()) {
            emit DispatcherUi::get()->loadSectorShape(fileName);
        }
    });
    auto load_kml = map_menu->addAction("Load KML");
    connect(load_kml, &QAction::triggered, [=](){
        QString fileName = QFileDialog::getOpenFileName(this, "Load KML", appConfig()->value("FLIGHT_PLANS_PATH").toString(), "KML files (*.kml)");
        if(!fileName.isEmpty()) {
            emit DispatcherUi::get()->loadKmlShape(fileName);
        }
    });


    // Edit Menu (Flight plan editing)
    auto fp_menu = menu_bar->addMenu("Edit");
    fp_menu->menuAction()->setVisible(false); // Hidden by default

    auto new_fp = fp_menu->addAction("New flight plan");
    new_fp->setShortcut(QKeySequence("N"));

    auto open_fp = fp_menu->addAction("Open flight plan");
    open_fp->setShortcut(QKeySequence("O"));
    connect(open_fp, &QAction::triggered, [=](){
        auto settings = getAppSettings();
        auto pprz_home = appConfig()->value("PAPARAZZI_HOME").toString();
        auto files = QFileDialog::getOpenFileNames(this, "open fp", pprz_home + "/conf/flight_plans", "*.xml");
        if(files.size() > 0) {
            for(auto &fp_file: files) {
                auto name = fp_file.split("/").last();
                AircraftManager::get()->addFPAircraft(name, fp_file);
            }
        }
    });

    auto save_fp = fp_menu->addAction("Save flight plan");
    // Removed duplicate S to prevent shortcut conflict

    auto close_fp = fp_menu->addAction("Close flight plan");
    close_fp->setShortcut(QKeySequence("W"));


    // Aircrafts placeholder is removed because A/Cs are appended at top level.

    // Help Menu (Right justified ideally)
    auto help_menu = menu_bar->addMenu("Help");

    auto keys_help = help_menu->addAction("Keys");
    connect(keys_help, &QAction::triggered, [=]() {
        QMessageBox::about(this, "Keys",
            "Zoom: Mouse Wheel, PgUp, PgDown, +, -\n"
            "Pan: Map & keyboard arrows\n"
            "Fit to window: f\n"
            "Center active A/C: c or C\n"
            "Fullscreen: F11\n"
            "Load Map Tile: Right\n"
            "Create Waypoint: Ctrl-Left\n"
            "Move Waypoint: Left drag\n"
            "Edit Waypoint: Left click\n"
        );
    });
    auto about = help_menu->addAction("About");
    QString about_txt =  QString(
            "<h1>Equinox GCS</h1>"
            "version %1"
            "<p>Equinox GCS is a ground control station application that can be used for <a href=\"https://paparazziuav.org\">paparrazi UAV</a> drones.</p>"
            "<p>source code can be found here: "
            "<a href=\"https://github.com/openuas/equinoxgcs\">https://github.com/openuas/equinoxgcs</a></p>").arg(QCoreApplication::applicationVersion());
    connect(about, &QAction::triggered, [=]() {
        QMessageBox::about(this,"About Equinox GCS", about_txt);
    });
}


void PprzMain::newAC(QString ac_id) {
    QString acName = AircraftManager::get()->getAircraft(ac_id)->name();
    
    // In legacy GCS, A/C menus are added to the main menu bar directly
    auto menu = menu_bar->addMenu(acName); // Removed the "&"
    
    auto fp_show = menu->addAction("Flight Plan");
    fp_show->setCheckable(true);
    fp_show->setChecked(true);
    
    menu->addAction("Center A/C", [=](){
        if(AircraftManager::get()->aircraftExists(ac_id)) {
            auto ac = AircraftManager::get()->getAircraft(ac_id);
            auto orig = ac->getFlightPlan()->getOrigin();
            Point2DLatLon pos(orig);
            if(ac->isReal()) {
                pos = ac->getPosition();
            }
            emit DispatcherUi::get()->centerMap(pos);
        }
    });
    menu->addAction("Clear Track", [=](){
        emit DispatcherUi::get()->clearTrackForAc(ac_id);
    });
    menu->addAction("Resize Track", [this, ac_id](){
        bool ok;
        int current_size = getAppSettings().value("map/aircraft/track_size", 500).toInt();
        emit DispatcherUi::get()->getTrackSizeForAc(ac_id, current_size);
        
        int size = QInputDialog::getInt(this, "Resize Track", QString("New track length for AC %1:").arg(ac_id), current_size, 0, 100000, 1, &ok);
        if (ok) {
            emit DispatcherUi::get()->resizeTrackForAc(ac_id, size);
        }
    });
    menu->addAction("Reset Waypoints", [=](){});

    auto sm = menu->addMenu("Datalink");
    sm->addAction("Jump to block", [=](){});
    sm->addAction("Commit Moves", [=](){});

    auto cam = menu->addAction("Cam footprint");
    cam->setCheckable(true);

    auto params = menu->addAction("A/C label");
    params->setCheckable(true);
    params->setChecked(true); // Label is shown by default
    connect(params, &QAction::toggled, [=](bool checked) {
        emit DispatcherUi::get()->toggleAcLabel(ac_id, checked);
    });
    params->setChecked(true); // Label is shown by default
    connect(params, &QAction::toggled, [=](bool checked) {
        emit DispatcherUi::get()->toggleAcLabel(ac_id, checked);
    });

    acMenus[ac_id] = menu;
    acActions[ac_id] = menu->menuAction();
}

void PprzMain::removeAC(QString ac_id) {
    auto action = acActions[ac_id];
    auto menu = acMenus[ac_id];

    acActions.remove(ac_id);
    acMenus.remove(ac_id);

    menu_bar->removeAction(action);

    menu->deleteLater();
    action->deleteLater();
}

void PprzMain::closeEvent(QCloseEvent *event) {
    if (GlobalConfig::get()->value("CONFIRM_QUIT", false).toBool()) {
        QMessageBox::StandardButton reply;
        reply = QMessageBox::question(this, "Leaving GCS", "Do you want to quit ?",
                                      QMessageBox::Yes | QMessageBox::No);
        if (reply == QMessageBox::No) {
            event->ignore();
            return;
        }
    }
    event->accept();
}
