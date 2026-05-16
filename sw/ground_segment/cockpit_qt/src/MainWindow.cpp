#include <QProcess>
#include <QDockWidget>
#include <QStatusBar>
#include <QLabel>
#include <QAction>
#include <QTabWidget>
#include <QPushButton>
#include <QToolBar>
#include <QDoubleSpinBox>

#include "gcs_utils.h"
#include "globalconfig.h"
#include "MainWindow.h"
#include "mapwidget.h"
#include "AircraftManager.h"
#include "dispatcher_ui.h"
#include "tools/srtm_manager.h"
#include "tools/coordinatestransform.h"
#include "pprzmain.h"
#include "maputils.h"
#include "point2dpseudomercator.h"

#include "widgets/widget_utils.h"
#include "widgets/link_status.h"
#include "widgets/infrared_viewer.h"
#include "widgets/misc_viewer.h"
#include "widgets/settings_classic_viewer.h"
#include "widgets/alert_viewer.h"
#include "widgets/gps_classic_viewer.h"
#include "widgets/flightplan_viewerv2.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
    setWindowTitle("Equinox GCS");
    
    // Central Map
    m_map = new MapWidget(this);
    setCentralWidget(m_map);

    // Strip Dock with Scroll Area
    QDockWidget *stripDock = new QDockWidget("Aircraft", this);
    stripDock->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetClosable);
    
    QScrollArea *scrollArea = new QScrollArea(this);
    scrollArea->setWidgetResizable(true);
    scrollArea->setFrameShape(QFrame::NoFrame);
    
    m_stripContainer = new QWidget();
    m_stripLayout = new QVBoxLayout(m_stripContainer);
    m_stripLayout->setContentsMargins(0, 0, 0, 0);
    m_stripLayout->setSpacing(2);
    m_stripLayout->addStretch();
    
    scrollArea->setWidget(m_stripContainer);
    stripDock->setWidget(scrollArea);
    addDockWidget(Qt::RightDockWidgetArea, stripDock);

    // Alerts Dock
    QDockWidget *alertDock = new QDockWidget("Alerts", this);
    alertDock->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetClosable);
    AlertViewer *globalAlerts = new AlertViewer(alertDock);
    alertDock->setWidget(globalAlerts);
    addDockWidget(Qt::RightDockWidgetArea, alertDock);
    splitDockWidget(stripDock, alertDock, Qt::Vertical);

    // Information Dock (Tabs)
    QDockWidget *infoDock = new QDockWidget("Information", this);
    m_infoTabs = new QTabWidget(infoDock);
    m_infoTabs->setObjectName("infoTabs");
    
    infoDock->setWidget(m_infoTabs);
    addDockWidget(Qt::BottomDockWidgetArea, infoDock);
    
    // Add status bar
    m_posLabel = new QLabel(this);
    m_posLabel->setMinimumWidth(150); // Give it some initial width so it doesn't collapse
    m_srtmLabel = new QLabel(this);
    m_srtmLabel->setMinimumWidth(40);
    
    m_zoomSpinBox = new QDoubleSpinBox(this);
    m_zoomSpinBox->setRange(1.0, 25.0);
    m_zoomSpinBox->setSingleStep(0.5);
    m_zoomSpinBox->setValue(m_map->zoom());
    
    // Connect spinbox to map zoom
    connect(m_zoomSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), m_map, &MapWidget::setZoom);
    // Connect map zoom to spinbox
    connect(m_map, &MapWidget::zoomChanged, m_zoomSpinBox, &QDoubleSpinBox::setValue);

    //statusBar()->showMessage("Ready");
    statusBar()->hide();

    // Connect AC signals
    connect(DispatcherUi::get(), &DispatcherUi::new_ac_config, this, &MainWindow::addAC);
    connect(DispatcherUi::get(), &DispatcherUi::ac_deleted, this, &MainWindow::removeAC);
    connect(DispatcherUi::get(), &DispatcherUi::ac_selected, this, &MainWindow::handleACSelected);

    // Map mouse move signal
    connect(m_map, &MapWidget::mouseMoved, this, &MainWindow::handleMouseMove);

    // SRTM action ground altitude extraction from srtm data at mouse position, with on-demand tile loading
    // Display SRTM alt at pointer position (will request for download if not available)
    m_displaySrtmAction = new QAction("Display SRTM alt", this);
    m_displaySrtmAction->setCheckable(true);
    if (appConfig()->value("SRTM_ENABLED").toBool()) {
        m_displaySrtmAction->setChecked(true);
    }
    
    // Defer adding to main menu to wait for PprzMain to initialize it
    connect(pprzApp()->mainWindow(), &PprzMain::ready, [this]() {
        QMenu* navMenu = nullptr;
        for (auto action : pprzApp()->mainWindow()->menuBar()->actions()) {
            if (action->menu() && action->menu()->title() == "Nav") {
                navMenu = action->menu();
                break;
            }
        }
        if (navMenu) {
            QAction* placeholder = nullptr;
            for (auto a : navMenu->actions()) {
                if (a->text().toLower() == "display srtm alt") {
                    placeholder = a;
                    break;
                }
            }
            if (placeholder) {
                m_displaySrtmAction->setText(placeholder->text());
                navMenu->insertAction(placeholder, m_displaySrtmAction);
                navMenu->removeAction(placeholder);
                delete placeholder;
            } else {
                navMenu->addAction(m_displaySrtmAction);
            }
        }
        
        auto dl_strm = [this](bool local_only) {
            QSet<QString> tiles;
            for (auto ac : AircraftManager::get()->getAircrafts()) {
                if (ac->getFlightPlan()) {
                    auto [nw, se] = ac->getFlightPlan()->boundingBox();
                    auto tile_names = SRTMManager::get()->get_tile_names(se.lat(), nw.lat(), nw.lon(), se.lon());
#if QT_VERSION_MAJOR == 5 && QT_VERSION_MINOR < 15
                    tiles.unite(tile_names.toSet());
#else
                    auto tiles_set = QSet<QString>(tile_names.begin(), tile_names.end());
                    tiles.unite(tiles_set);
#endif
                }
            }
            Point2DLatLon nw(0, 0), se(0, 0);
            m_map->getViewPoints(nw, se);
            auto tile_names = SRTMManager::get()->get_tile_names(se.lat(), nw.lat(), nw.lon(), se.lon());
#if QT_VERSION_MAJOR == 5 && QT_VERSION_MINOR < 15
            tiles.unite(tile_names.toSet());
#else
            auto tiles_set = QSet<QString>(tile_names.begin(), tile_names.end());
            tiles.unite(tiles_set);
#endif
            SRTMManager::get()->load_tiles(tiles.values(), local_only);
        };

        connect(m_displaySrtmAction, &QAction::toggled, this, [dl_strm](bool checked){
            if (checked) {
                dl_strm(false);
            }
        });
        
        QToolBar* mainToolBar = pprzApp()->mainWindow()->findChild<QToolBar*>("mainToolBar");
        if (mainToolBar) {
            QWidget* spacer = new QWidget(mainToolBar);
            spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
            mainToolBar->addWidget(spacer);

            QAction* toggleMapAction = new QAction("Map", this);
            toggleMapAction->setCheckable(true);
            toggleMapAction->setChecked(true);
            connect(toggleMapAction, &QAction::toggled, this, [this](bool checked) {
                m_map->setMapVisible(checked);
            });
            mainToolBar->addAction(toggleMapAction);
            if (navMenu) {
                for (auto a : navMenu->actions()) {
                    if (a->text() == "Background") {
                        a->setCheckable(true);
                        a->setChecked(toggleMapAction->isChecked());
                        connect(toggleMapAction, &QAction::toggled, a, &QAction::setChecked);
                        connect(a, &QAction::toggled, toggleMapAction, &QAction::setChecked);
                        break;
                    }
                }
            }
            mainToolBar->addSeparator();

            m_posLabel->setParent(mainToolBar);
            mainToolBar->addWidget(m_posLabel);
            mainToolBar->addSeparator();

            m_srtmLabel->setParent(mainToolBar);
            mainToolBar->addWidget(m_srtmLabel);
            mainToolBar->addSeparator();

            m_zoomSpinBox->setParent(mainToolBar);
            mainToolBar->addWidget(m_zoomSpinBox);
        }
    });

    connect(m_infoTabs, &QTabWidget::currentChanged, this, [this](int index) {
        if (index >= 0) {
            QWidget *w = m_infoTabs->widget(index);
            for (auto it = m_acTabWidgets.begin(); it != m_acTabWidgets.end(); ++it) {
                if (it.value() == w) {
                    handleACSelected(it.key());
                    break;
                }
            }
        }
    });

// Initialize existing aircraft
    for(auto ac : AircraftManager::get()->getAircrafts()) {
        addAC(ac->getId());
    }

    if (appConfig() && appConfig()->contains("MPLAYER") && !appConfig()->value("MPLAYER").toString().isEmpty()) {
        QDockWidget *dock = new QDockWidget("Video Player", this);
        QWidget* videoWidget = new QWidget(dock);
        videoWidget->setStyleSheet("background-color: black;");
        dock->setWidget(videoWidget);
        addDockWidget(Qt::RightDockWidgetArea, dock);
        
        QProcess *process = new QProcess(this);
        QStringList args;
        args << "-wid" << QString::number(videoWidget->winId());
        
        #if QT_VERSION >= QT_VERSION_CHECK(5, 14, 0)
            args.append(appConfig()->value("MPLAYER").toString().split(" ", Qt::SkipEmptyParts));
        #else
            args.append(appConfig()->value("MPLAYER").toString().split(" ", QString::SkipEmptyParts));
        #endif
        
        process->start("mplayer", args);
    }
    if (appConfig() && appConfig()->contains("PLUGIN") && !appConfig()->value("PLUGIN").toString().isEmpty()) {
        QDockWidget *dock = new QDockWidget("Plugin Application", this);
        QWidget* pWidget = new QWidget(dock);
        pWidget->setStyleSheet("background-color: black;");
        dock->setWidget(pWidget);
        addDockWidget(Qt::RightDockWidgetArea, dock);
        
        QString pluginStr = appConfig()->value("PLUGIN").toString();
        // The plugin command is expected to launch with the window ID appended.
        // e.g. "vlc --drawable-xid=" + winId
        QString command = pluginStr + QString::number(pWidget->winId());
        
        QProcess *process = new QProcess(this);
        process->start("sh", QStringList() << "-c" << command);
    }

    // Load static maps and display a calibrated (XML) map
    for (QString f : appConfig()->value("MAP_FILES").toStringList()) {
        if (f.endsWith(".kml", Qt::CaseInsensitive)) {
            emit DispatcherUi::get()->loadKmlShape(f);
        } else {
            qDebug() << "XML map files not yet implemented in Qt C++ Cockpit for file:" << f;
        }
    }
}

MainWindow::~MainWindow() {}

void MainWindow::addAC(QString ac_id) {
    if (Strip::getStrip(ac_id)) return;
    
    Strip *strip = new Strip(ac_id, m_stripContainer, true);
    m_stripLayout->insertWidget(m_stripLayout->count() - 1, strip);

    // Create the inner tab widget for this aircraft
    QTabWidget *acTabs = new QTabWidget(m_infoTabs);
    acTabs->setObjectName("acTabs_" + ac_id);
    
    if (AircraftManager::get()->getAircraft(ac_id)) {
        QColor ac_color = AircraftManager::get()->getAircraft(ac_id)->getColor();
        int hue = ac_color.hue();
        int sat = ac_color.saturation();
        ac_color.setHsv(hue, static_cast<int>(sat*0.2), 255);
        QString style = QString(
            "QTabWidget::pane { background: %1; border: 1px solid %2; } "
            "QTabBar::tab:selected { background: %1; }"
            "QWidget#%3 { background: %1; }" 
        ).arg(ac_color.name(), ac_color.darker(150).name(), acTabs->objectName());
        acTabs->setStyleSheet(style);
    }
    
    // Add Link, Horizon, Alerts, Infrared, Misc specifically for this Aircraft
    auto wrapInScroll = [acTabs](QWidget* w) {
        QScrollArea *sa = new QScrollArea(acTabs);
        sa->setWidgetResizable(true);
        sa->setFrameShape(QFrame::NoFrame);
        sa->setWidget(w);
        return sa;
    };

    FlightPlanViewerV2 *flightplan = new FlightPlanViewerV2(ac_id, acTabs);
    acTabs->addTab(flightplan, "Flightplan");

    InfraredViewer *ir = new InfraredViewer(ac_id, acTabs);
    int ir_idx = acTabs->addTab(wrapInScroll(ir), "Infrared");
#if QT_VERSION >= QT_VERSION_CHECK(5, 15, 0)
    acTabs->setTabVisible(ir_idx, false);
#else
    acTabs->removeTab(ir_idx);
#endif

    GPSViewerTab *gpsTab = new GPSViewerTab(ac_id, acTabs);
    acTabs->addTab(gpsTab, "GPS");
    
    HorizonViewer *horizon = new HorizonViewer(acTabs);
    horizon->setAttitude(0.0, 0.0);
    // Disconnect global selection and fix this instance to the given AC
    disconnect(DispatcherUi::get(), &DispatcherUi::ac_selected, horizon, &HorizonViewer::changeCurrentAC);
    horizon->changeCurrentAC(ac_id);
    acTabs->addTab(horizon, "PFD");
    
    LinkStatus *link = new LinkStatus(ac_id, acTabs);
    acTabs->addTab(wrapInScroll(link), "Link");
    
    MiscViewer *misc = new MiscViewer(ac_id, acTabs);
    acTabs->addTab(wrapInScroll(misc), "Misc");

    SettingsClassicViewer *settingsTab = new SettingsClassicViewer(ac_id, acTabs);
    acTabs->addTab(settingsTab, "Settings");

    QString display_name = "AC " + ac_id;
    QIcon ac_icon;
    if (AircraftManager::get()->getAircraft(ac_id)) {
        display_name = QString("%1 (%2)").arg(AircraftManager::get()->getAircraft(ac_id)->name(), ac_id);
        
        QColor icon_color = AircraftManager::get()->getAircraft(ac_id)->getColor();
        int hue = icon_color.hue();
        int sat = icon_color.saturation();
        icon_color.setHsv(hue, static_cast<int>(sat*0.4), 255); // slightly stronger for small icon
        QPixmap pix(12, 12);
        pix.fill(icon_color);
        ac_icon = QIcon(pix);
    }
    m_infoTabs->addTab(acTabs, ac_icon, display_name);
    m_acTabWidgets[ac_id] = acTabs;
}

void MainWindow::removeAC(QString ac_id) {
    Strip *strip = Strip::getStrip(ac_id);
    if (strip) {
        m_stripLayout->removeWidget(strip);
        strip->deleteLater();
    }
    
    if (m_acTabWidgets.contains(ac_id)) {
        QWidget *w = m_acTabWidgets[ac_id];
        m_infoTabs->removeTab(m_infoTabs->indexOf(w));
        w->deleteLater();
        m_acTabWidgets.remove(ac_id);
    }
}

void MainWindow::handleMouseMove(QPointF scenePos) {
    auto tp = tilePoint(scenePos, zoomLevel(m_map->zoom()), m_map->tileSize());
    Point2DPseudoMercator ppm(tp);
    auto pt = CoordinatesTransform::get()->pseudoMercator_to_WGS84(ppm);

    // Always update position label
    m_posLabel->setText(pt.toString());

    if (m_displaySrtmAction->isChecked()) {
        auto elevation = SRTMManager::get()->get_elevation(pt.lat(), pt.lon());
        if (elevation) {
            m_srtmLabel->setText(QString("SRTM: %1m").arg(elevation.value()));
        } else {
            m_srtmLabel->setText("SRTM: N/A");
        }
    } else {
        m_srtmLabel->clear();
    }
}

void MainWindow::handleACSelected(QString ac_id) {
    if (m_acTabWidgets.contains(ac_id)) {
        QWidget *w = m_acTabWidgets[ac_id];
        m_infoTabs->setCurrentWidget(w);
        
        auto ac = AircraftManager::get()->getAircraft(ac_id);
        if (ac) {
            QColor ac_color = ac->getColor();
            int hue = ac_color.hue();
            int sat = ac_color.saturation();
            ac_color.setHsv(hue, static_cast<int>(sat*0.2), 255);
            QString const c = ac_color.name();
            QString dark_c = ac_color.darker(150).name();

            QString style = QString(
                "QTabWidget#infoTabs::pane { background: %1; border: 1px solid %2; }"
                "QTabWidget#infoTabs > QTabBar::tab:selected { background: %1; }"
            ).arg(c, dark_c);
            m_infoTabs->setStyleSheet(style);
        }
    }
}



/* --- HISTORICAL OCAML COMMENTS: gcs.ml ---
-
 * Compatibility with the old UTM format
 ------------------------------------------
 * Take this point as a reference for the display if none currently
 ------------------------------------------
 * * Save the given pixbuf calibrated with NW and SE corners
 ------------------------------------------
 * ***** Creates a calibrated map from the bitmap (selected region) **********
 ------------------------------------------
 * * This module could be inserted into Ocaml_toosl; but it requires threads.cma
 ------------------------------------------
 * * A list of functions to call
 ------------------------------------------
 * * The id of a running thread executing the queue
 ------------------------------------------
 * * A mutex to handle concurrent accesses
 ------------------------------------------
 * * Nothing mode to do: exiting the thread
 ------------------------------------------
 * * Pick a function from the list, call it and continue
 ------------------------------------------
 * * Add the function to the queue
 ------------------------------------------
 * * Nobody is currently running the queue: start a thread
 ------------------------------------------
 * *********** Maps handling (Google, OSM, MS, etc.) **********************************
 ------------------------------------------
 * * Fill the visible background with map tiles
 ------------------------------------------
 * * Creates a calibrated map from the map tiles (selected region)
 ------------------------------------------
 * GM module
 ------------------------------------------
 * * First estimate the coverage of the window
 ------------------------------------------
 * ****** Mouse motion handling *********************************************
 ------------------------------------------
 * ****** Mouse wheel handling **********************************************
 ------------------------------------------
 * ****** Mouse buttons handling *********************************************
 ------------------------------------------
 * * Display a map tile from map provider (Google, OSC, ..) or IGN
 ------------------------------------------
 * create new wp on Ctrl-click
 ------------------------------------------
 * ******* Help **************************************************************
 ------------------------------------------
 * **************** MAIN *****************************************************
 ------------------------------------------
 * Maps handling
 ------------------------------------------
 * Choose the map source
 ------------------------------------------
 * Determine a decent default selected item
 ------------------------------------------
 * Choose the map policy
 ------------------------------------------
 * Determine a decent default selected item
 ------------------------------------------
 * Map tiles fill menu entry and toolbar button
 ------------------------------------------
 * * Connect Maps display to view change
 ------------------------------------------
 * * Flight plan editing
 ------------------------------------------
 * * Help pushed to the right
 ------------------------------------------
 * * Separate from A/C menus
 ------------------------------------------
 * * Set the initial zoom
 ------------------------------------------
 * get DTD head line for layout
 ------------------------------------------
 * ************************* MAIN *******************************************
 ------------------------------------------
 * * The whole window map2d *
 ------------------------------------------
 * Editor frame
 ------------------------------------------
 * * Put the canvas in a frame
 ------------------------------------------
 * * window for the strip panel
 ------------------------------------------
 * * Aircraft notebook
 ------------------------------------------
 * * Alerts text frame
 ------------------------------------------
 * * Altitude graph frame
 ------------------------------------------
 * * plugin frame
 ------------------------------------------
 * * packing papgets
 ------------------------------------------
 * Ask if ac_id parameters from papgets should be saved
 ------------------------------------------
 * * Keep the center of the geo canvas
 ------------------------------------------
 * * Wait for A/Cs and subsequent messages
 ------------------------------------------
 * * Display the window
 ------------------------------------------
 * * Loading an initial map
 ------------------------------------------
 * * Center the map as required
 ------------------------------------------
 * * Threaded main loop (map tiles loaded concurrently)
 ------------------------------------------
*/
