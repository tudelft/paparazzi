// Real time handling of flying A/Cs

#include <QDebug>
#include <QDir>
#include <QCommandLineParser>
#include <QCommandLineOption>
#include <QProcess>
#include "PprzApplication.h"
#include "pprzmain.h"
#include "MainWindow.h"
#include "configurator.h"
#include "common/gcs_utils.h"
#include "globalconfig.h"
#include "app_settings.h"
#include "Live.h"
#include "tools/pprz_dispatcher.h"
#include "Particules.h"
#include "Intruders.h"
#include "Shapes.h"


int main(int argc, char *argv[]) {
    PprzApplication app(argc, argv);
    app.setApplicationName("Equinox GCS");
    app.setApplicationVersion("1.0");

    // Initialize global paths BEFORE creating the PprzMain / widgets
    // but AFTER the application object is created so applicationDirPath is available.
    auto config = GlobalConfig::get();
    QString app_dir = QCoreApplication::applicationDirPath();
    config->setValue("APP_DATA_PATH", app_dir + "/data");
    config->setValue("USER_DATA_PATH", app_dir + "/data");
    config->setValue("SETTINGS_PATH", app_dir + "/data/settings.ini");
    config->setValue("MESSAGES", app_dir + "/data/messages.xml");
    config->setValue("IVY_BUS", "127.255.255.255:2010");
    config->setValue("LAYOUT_FILE", "default_layout.xml");

    static const int InitialWindowWidth = 800;
    static const int InitialWindowHeight = 600;
    static const double InitialZoom = 17.0;
    static const char InitialWindowInfo[] = "Classic GCS";

    QCommandLineParser parser;
    parser.setSingleDashWordOptionMode(QCommandLineParser::ParseAsLongOptions);
    parser.setApplicationDescription("Equinox GCS");
    parser.addHelpOption();
    parser.addVersionOption();

    // Define all classic GCS options
    QCommandLineOption autoOrthoOption("auto_ortho", "IGN tiles path", "path");
    QCommandLineOption ivyBusOption("b", "Ivy bus (default 127.255.255.255:2010)", "bus", "127.255.255.255:2010");
    QCommandLineOption centerOption("center", "Initial map center (e.g. 'WGS84 43.605 1.443')", "center");
    QCommandLineOption centerAcOption("center_ac", "Centers the map on any new A/C");
    QCommandLineOption centerAcIdOption("center_ac_id", "Continuously centers the map on the AC id", "ac_id");
    QCommandLineOption editOption("edit", "Flight plan editor mode");
    QCommandLineOption fullscreenOption("fullscreen", "Fullscreen window");
    QCommandLineOption mapsFillOption("maps_fill", "Automatically start loading background maps");
    QCommandLineOption mapsZoomOption("maps_zoom", "Background maps zoomlevel (18-22)", "level", "18");
    QCommandLineOption ignOption("ign", "IGN tiles path", "path");
    QCommandLineOption lambertIIeOption("lambertIIe", "Switch to LambertIIe projection");
    QCommandLineOption layoutOption("layout", "GUI layout XML specification", "file", "default_layout.xml");
    QCommandLineOption mOption("m", "Map XML description file", "file");
    QCommandLineOption maximizeOption("maximize", "Maximize window");
    QCommandLineOption mercatorOption("mercator", "Switch to Mercator projection (default)");
    QCommandLineOption mplayerOption("mplayer", "Launch mplayer with the given argument as X plugin", "args");
    QCommandLineOption noAlarmOption("no_alarm", "Disables alarm page");
    QCommandLineOption mapsNoHttpOption("maps_no_http", "No HTTP for maps, use cache only");
    QCommandLineOption orthoOption("ortho", "IGN tiles path", "path");
    QCommandLineOption osmOption("osm", "Use OpenStreetMap database (default is Google)");
    QCommandLineOption msOption("ms", "Use Microsoft maps database (default is Google)");
    QCommandLineOption particulesOption("particules", "Display particules");
    QCommandLineOption pluginOption("plugin", "External X application", "string");
    QCommandLineOption refOption("ref", "Geographic ref (e.g. 'WGS84 43.605 1.443')", "ref");
    QCommandLineOption speechOption("speech", "Enable vocal messages");
    QCommandLineOption srtmOption("srtm", "Enable SRTM elevation display");
    QCommandLineOption trackSizeOption("track_size", "Default track length", "size", "500");
    QCommandLineOption utmOption("utm", "Switch to UTM local projection");
    QCommandLineOption widOption("wid", "Id of an existing window to be attached to", "window_id");
    QCommandLineOption zoomOption("zoom", "Initial zoom", "value");
    QCommandLineOption autoHideFpOption("auto_hide_fp", "Automatically hide flight plans of unselected aircraft");
    QCommandLineOption timestampOption("timestamp", "Bind on timestamped telemetry messages");
    QCommandLineOption acIdsOption("ac_ids", "Comma separated list of AC IDs to show in GCS", "ids");
    QCommandLineOption noConfirmKillOption("no_confirm_kill", "Disable kill confirmation from strip button");
    QCommandLineOption confirmQuitOption("confirm_quit", "Confirm before quitting");

    parser.addOptions({
        autoOrthoOption, ivyBusOption, centerOption, centerAcOption, centerAcIdOption,
        editOption, fullscreenOption, mapsFillOption, mapsZoomOption, ignOption,
        lambertIIeOption, layoutOption, mOption, maximizeOption, mercatorOption,
        mplayerOption, noAlarmOption, mapsNoHttpOption, orthoOption, osmOption,
        msOption, particulesOption, pluginOption, refOption, speechOption,
        srtmOption, trackSizeOption, utmOption, widOption, zoomOption,
        autoHideFpOption, timestampOption, acIdsOption, noConfirmKillOption,
        confirmQuitOption
    });

    parser.process(app);

    // Apply functional options
    if (parser.isSet(ivyBusOption)) {
        config->setValue("IVY_BUS", parser.value(ivyBusOption));
    }
    if (parser.isSet(layoutOption)) {
        config->setValue("LAYOUT_FILE", parser.value(layoutOption));
    } else if (parser.isSet(editOption)) {
        config->setValue("LAYOUT_FILE", "fp_editor_layout.xml");
    } else if (parser.isSet(editOption)) {
        config->setValue("LAYOUT_FILE", "fp_editor_layout.xml");
    }
    if (parser.isSet(speechOption)) {
        config->setValue("SPEECH_ENABLED", true);
#if defined(Q_OS_MAC)
        QProcess::startDetached("say", QStringList() << "Welcome to papa ratsi");
#else
        QProcess::startDetached("espeak", QStringList() << "Welcome to papa ratsi");
#endif
    }
    if (parser.isSet(autoHideFpOption)) {
        config->setValue("AUTO_HIDE_FP", true);
    }
    if (parser.isSet(noConfirmKillOption)) {
        config->setValue("CONFIRM_KILL", false);
    }
    if (parser.isSet(confirmQuitOption)) {
        config->setValue("CONFIRM_QUIT", true);
    }

    // Map & Background settings
    config->setValue("AUTO_ORTHO", parser.value(autoOrthoOption));
    config->setValue("MAPS_FILL", parser.isSet(mapsFillOption));
    config->setValue("MAPS_ZOOM", parser.value(mapsZoomOption).toInt());
    config->setValue("MAPS_NO_HTTP", parser.isSet(mapsNoHttpOption));
    config->setValue("IGN_PATH", parser.value(ignOption));
    config->setValue("ORTHO_PATH", parser.value(orthoOption));

    // Map provider configuration
    if (parser.isSet(osmOption)) {
        config->setValue("MAPS_SOURCE", "osm");
    } else if (parser.isSet(msOption)) {
        config->setValue("MAPS_SOURCE", "ms");
    }

    // Projection
    if (parser.isSet(lambertIIeOption)) {
        config->setValue("PROJECTION", "lambertIIe");
    } else if (parser.isSet(utmOption)) {
        config->setValue("PROJECTION", "utm");
    } else if (parser.isSet(mercatorOption)) {
        config->setValue("PROJECTION", "mercator");
    } else {
        config->setValue("PROJECTION", "mercator"); // Default
    }

    // Position & sizing
    config->setValue("INITIAL_CENTER", parser.value(centerOption));
    config->setValue("MAP_TRACK_AC", parser.isSet(centerAcOption));
    config->setValue("CENTER_AC_ID", parser.value(centerAcIdOption));
    if (parser.isSet(zoomOption)) {
        config->setValue("INITIAL_ZOOM", parser.value(zoomOption).toFloat());
    }
    config->setValue("GEO_REF", parser.value(refOption));
    
    // Application & feature toggles
    config->setValue("EDIT_MODE", parser.isSet(editOption));
    config->setValue("NO_ALARM", parser.isSet(noAlarmOption));
    config->setValue("PARTICULES", parser.isSet(particulesOption));
    config->setValue("SRTM_ENABLED", parser.isSet(srtmOption));
    config->setValue("TIMESTAMP", parser.isSet(timestampOption));
    
    // Window embedding / external apps
    config->setValue("WINDOW_ID", parser.value(widOption));
    config->setValue("PLUGIN", parser.value(pluginOption));
    config->setValue("MPLAYER", parser.value(mplayerOption));

    // Other settings
    config->setValue("TRACK_SIZE", parser.value(trackSizeOption).toInt());
    config->setValue("AC_IDS", parser.value(acIdsOption));
    
    // Maps list
    config->setValue("MAP_FILES", parser.values(mOption));

    set_app_settings();
    app.init();
    Live::get()->init();
    if (appConfig()->value("PARTICULES").toBool()) { Particules::get()->init(); }
    Shapes::get()->init();
    Intruders::get()->init();

    // Create the cockpit central widget with our custom layout
    MainWindow *cockpit = new MainWindow();

    // Setup the PprzMain window with our cockpit as central widget
    PprzMain *mainWin = pprzApp()->mainWindow();
    mainWin->setupUi(InitialWindowWidth, InitialWindowHeight, cockpit);
    mainWin->setWindowTitle(InitialWindowInfo);

    // Apply window state options
    if (parser.isSet(fullscreenOption)) {
        mainWin->showFullScreen();
    } else if (parser.isSet(maximizeOption)) {
        mainWin->showMaximized();
    } else {
        mainWin->show();
    }

    // Start IVY dispatcher, kind of Live.ml on previous GCS but with a more direct integration to Qt signals/slots and the rest of the application
    PprzDispatcher::get()->start();

    return app.exec();
}

