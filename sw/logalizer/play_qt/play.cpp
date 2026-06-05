// Re-implementation of Paparazzi Play (OCaml) in C++ (Qt6)

#include <QApplication>
#include <QMainWindow>
#include <QCommandLineParser>
#include <QDebug>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QSlider>
#include <QLabel>
#include <QTimer>
#include <QFile>
#include <QTextStream>
#include <QXmlStreamReader>
#include <QDomDocument>
#include <QDir>
#include <QStandardPaths>
#include <QProcess>
#include <QFileDialog>
#include <QDoubleSpinBox>
#include <cmath>

#include <IvyQt/ivyqt.h>
#include <pprzlinkQt/MessageDictionary.h>

#include "../../include/linux_desktop_utils.h"

// Define RAII StderrBlocker to suppress GTK warnings when using native dialogs (same as before)
#include <unistd.h>
#include <fcntl.h>
class StderrBlocker {
    int oldStderr;
    int devNull;
public:
    StderrBlocker() {
        fflush(stderr);
        oldStderr = dup(STDERR_FILENO);
        devNull = open("/dev/null", O_WRONLY);
        if (devNull >= 0) dup2(devNull, STDERR_FILENO);
    }
    ~StderrBlocker() {
        fflush(stderr);
        if (devNull >= 0) {
            dup2(oldStderr, STDERR_FILENO);
            close(devNull);
        }
        close(oldStderr);
    }
};

struct LogEntry {
    double time;
    QString ac;
    QString msg; // e.g. "BAT 12.5 120 ..."
    QString msgName; // e.g. "BAT"
};

class PlayCore : public QObject {
    Q_OBJECT
public:
    PlayCore(QObject* parent = nullptr) : QObject(parent),
        m_speed(1.0),
        m_currentIndex(0),
        m_isPlaying(false),
        m_bus(nullptr)
    {
        m_timer = new QTimer(this);
        m_timer->setSingleShot(true);
        connect(m_timer, &QTimer::timeout, this, &PlayCore::onTimeout);
    }

    void setIvyBus(const QString& busArg) {
        m_ivyBusArg = busArg;
    }
    
    void setNoGui(bool noGui) {
        m_noGui = noGui;
    }

    bool loadLog(const QString& xmlFile) {
        m_xmlFile = xmlFile;
        QFile file(xmlFile);
        if (!file.open(QIODevice::ReadOnly)) {
            qWarning() << "Cannot open file:" << xmlFile;
            return false;
        }

        QDomDocument doc;
        if (!doc.setContent(&file)) {
            qWarning() << "Failed to parse XML:" << xmlFile;
            return false;
        }

        QDomElement root = doc.documentElement();
        
        // 1. Data file extract
        QString dataFileName;
        QDomNodeList dataFileNodes = root.elementsByTagName("data_file");
        if (!dataFileNodes.isEmpty()) {
            dataFileName = dataFileNodes.at(0).toElement().attribute("name");
        } else {
            // As fallback, check if root itself has data_file. The log structure is usually <log><data_file name="..."/></log>
            if (root.hasAttribute("data_file")) dataFileName = root.attribute("data_file");
        }

        if (dataFileName.isEmpty()) {
            qWarning() << "data_file not found in XML";
            return false;
        }

        QFileInfo fi(xmlFile);
        QString dataFilePath = fi.absoluteDir().filePath(dataFileName);
        
        if (!QFile::exists(dataFilePath)) {
            // Uncompress logic
            if (QFile::exists(dataFilePath + ".gz")) {
                QProcess::execute("gunzip", {dataFilePath + ".gz"});
            } else if (QFile::exists(dataFilePath + ".bz2")) {
                QProcess::execute("bunzip2", {dataFilePath + ".bz2"});
            }
        }

        // 2. Load the actual data
        QFile dataF(dataFilePath);
        if (!dataF.open(QIODevice::ReadOnly | QIODevice::Text)) {
            qWarning() << "Cannot open data file:" << dataFilePath;
            return false;
        }

        m_log.clear();
        QSet<QString> acs;
        QTextStream in(&dataF);
        QRegularExpression spaceRe("\\s+");
        
        while (!in.atEnd()) {
            QString line = in.readLine().trimmed();
            if (line.isEmpty()) continue;
            QStringList parts = line.split(spaceRe, Qt::SkipEmptyParts);
            if (parts.size() >= 3) {
                bool ok;
                double t = parts[0].toDouble(&ok);
                if (ok) {
                    QString ac = parts[1];
                    // reconstructed msg
                    QString msg = line.mid(line.indexOf(parts[2]));
                    QString msgName = parts[2];
                    acs.insert(ac);
                    m_log.push_back({t, ac, msg, msgName});
                }
            }
        }

        if (m_log.isEmpty()) {
            qWarning() << "No telemetry lines found.";
            return false;
        }

        // 3. Extract and store conf and protocol
        storeConf(root, acs);
        storeMessages(root);
        
        // 4. Initialize MessageDictionary to resolve Telemetry vs Ground
        initDictionary();

        m_currentIndex = 0;
        emit logLoaded(m_log.first().time, m_log.last().time);
        return true;
    }

    void startIvy() {
        QString pprzName = "Paparazzi replay";
        m_bus = new IvyQt(pprzName, "READY", this);
        // default 2010 but parsing domain out of m_ivyBusArg
        QString domain = m_ivyBusArg;
        int port = 2010;
        if (domain.contains(':')) {
            QStringList parts = domain.split(':');
            domain = parts[0];
            port = parts[1].toInt();
        }
                        m_bus->start(domain, port);
        
        m_bus->bindMessage("WORLD_ENV (.*)", [this](Peer*, QStringList params) {
            if (m_timeScaleIdx >= 0 && !params.isEmpty()) {
                QString data = params[0];
                QStringList fields = data.split(' ', Qt::SkipEmptyParts);
                if (m_timeScaleIdx < fields.size()) {
                    bool ok;
                    double scale = fields[m_timeScaleIdx].toDouble(&ok);
                    if (ok && scale > 0.0) {
                        emit speedChangedByNetwork(scale);
                    }
                }
            }
        });

    void setSpeed(double speed) {
        m_speed = speed;
        if (m_isPlaying) {
            play(); // Recalculate timer
        }
    }

    void setTime(double t) {
        // Find index via binary search
        auto it = std::lower_bound(m_log.begin(), m_log.end(), t, [](const LogEntry& a, double tVal) {
            return a.time < tVal;
        });
        m_currentIndex = std::distance(m_log.begin(), it);
        if (m_currentIndex >= m_log.size()) m_currentIndex = m_log.size() - 1;
        emit timeUpdated(m_log[m_currentIndex].time);
    }
    
    void getBounds(double& start, double& end) {
        if (!m_log.isEmpty()) {
           start = m_log.first().time;
           end = m_log.last().time;
        } else {
           start = 0; end = 0;
        }
    }

    void play() {
        if (m_log.isEmpty() || m_currentIndex >= m_log.size()) return;
        m_isPlaying = true;
        scheduleNext();
    }

    void stop() {
        m_isPlaying = false;
        m_timer->stop();
    }

signals:
    void logLoaded(double minT, double maxT);
    void timeUpdated(double currentT);
    void speedChangedByNetwork(double newSpeed);
    void finished();

private slots:
    void onTimeout() {
        if (!m_isPlaying || m_currentIndex >= m_log.size()) return;
        
        const LogEntry& entry = m_log[m_currentIndex];
        
        // Send Ivy logic
        if (m_bus) {
            if (m_groundMsgs.contains(entry.msgName)) {
                // Ground message
                m_bus->send(QString("replay_ground %1").arg(entry.msg));
            }
            if (m_telemetryMsgs.contains(entry.msgName) || !m_groundMsgs.contains(entry.msgName)) {
                // Telemetry message
                m_bus->send(QString("replay%1 %2").arg(entry.ac).arg(entry.msg));
                m_bus->send(QString("time%1 %2").arg(entry.ac).arg(QString::number(entry.time, 'f', 6)));
            }
        }

        emit timeUpdated(entry.time);
        
        m_currentIndex++;
        
        if (m_currentIndex < m_log.size()) {
            scheduleNext();
        } else {
            m_isPlaying = false;
            if (m_noGui) {
                emit finished();
                QCoreApplication::quit();
            }
        }
    }

private:
    void scheduleNext() {
        if (m_currentIndex + 1 < m_log.size()) {
            double dt = m_log[m_currentIndex + 1].time - m_log[m_currentIndex].time;
            int msTimer = std::max(0, static_cast<int>(std::round(1000.0 * dt / std::max(0.01, m_speed))));
            m_timer->start(msTimer);
        } else {
            // Start the last frame immediately if we can't schedule next
            m_timer->start(0); 
        }
    }

    void storeConf(const QDomElement& root, const QSet<QString>& acs) {
        QString pprzHome = qEnvironmentVariable("PAPARAZZI_HOME");
        if (pprzHome.isEmpty()) pprzHome = QDir::currentPath();
        
        QString replayDir = pprzHome + "/var/replay";
        QDir().mkpath(replayDir + "/conf");
        QDir().mkpath(replayDir + "/var/aircrafts");

        QDomNodeList confNodes = root.elementsByTagName("conf");
        if (confNodes.isEmpty()) return;
        QDomElement confEl = confNodes.at(0).toElement();

        QDomDocument outDoc;
        QDomElement outConf = outDoc.createElement("conf");
        outDoc.appendChild(outConf);

        QDomNode child = confEl.firstChild();
        while (!child.isNull()) {
            if (child.isElement() && child.nodeName() == "aircraft") {
                QDomElement acEl = child.toElement();
                QString acId = acEl.attribute("ac_id");
                if (acs.contains(acId)) {
                    QString acName = acEl.attribute("name");
                    QString acDirStr = replayDir + "/var/aircrafts/" + acName;
                    QDir acDir(acDirStr);
                    acDir.mkpath(".");
                    acDir.mkpath("conf");
                    
                    auto writeXmlFile = [](const QString& path, const QDomElement& el) {
                        QFile f(path);
                        if (f.open(QIODevice::WriteOnly | QIODevice::Text)) {
                            QTextStream out(&f);
                            el.save(out, 2);
                        }
                    };

                    auto extractChild = [&](const QString& tag) {
                        QDomNodeList list = acEl.elementsByTagName(tag);
                        if (!list.isEmpty()) {
                            QDomElement el = list.at(0).toElement();
                            writeXmlFile(replayDir + "/conf/" + acEl.attribute(tag), el);
                            writeXmlFile(acDirStr + "/conf/" + acEl.attribute(tag), el);
                        }
                    };

                    extractChild("airframe");
                    extractChild("radio");

                    QDomNodeList genSet = acEl.elementsByTagName("generated_settings");
                    if (!genSet.isEmpty()) {
                        QDomElement settingsXml = genSet.at(0).toElement();
                        writeXmlFile(acDirStr + "/settings.xml", settingsXml);
                        writeXmlFile(replayDir + "/settings.xml", settingsXml);
                    } else {
                        qWarning() << "Replay: no settings for display";
                        QDomElement dummy = outDoc.createElement("settings");
                        writeXmlFile(acDirStr + "/settings.xml", dummy);
                        writeXmlFile(replayDir + "/settings.xml", dummy);
                    }

                    bool orig_fp = !acEl.elementsByTagName("flight_plan").isEmpty();
                    if (orig_fp) {
                        extractChild("flight_plan");
                        QString fpName = acEl.attribute("flight_plan");
                        QString dumpFpProg = qEnvironmentVariable("PAPARAZZI_SRC") + "/sw/tools/generators/dump_flight_plan.out";
                        QString fpath = replayDir + "/conf/" + fpName;
                        QString dumpPath = acDirStr + "/flight_plan.xml";
                        QProcess::execute(dumpFpProg, {fpath, dumpPath});
                    } else {
                        QDomNodeList dumps = acEl.elementsByTagName("dump");
                        if (!dumps.isEmpty()) {
                            writeXmlFile(acDirStr + "/flight_plan.xml", dumps.at(0).toElement());
                        }
                    }

                    outConf.appendChild(acEl.cloneNode(true));
                }
            } else {
                // Keep ground section
                outConf.appendChild(child.cloneNode(true));
            }
            child = child.nextSibling();
        }

        QFile outF(replayDir + "/conf/conf.xml");
        if (outF.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QTextStream outT(&outF);
            outDoc.save(outT, 2);
        }
    }

    void storeMessages(const QDomElement& root) {
        QString pprzHome = qEnvironmentVariable("PAPARAZZI_HOME");
        if (pprzHome.isEmpty()) pprzHome = QDir::currentPath();
        QString replayDir = pprzHome + "/var/replay";
        QDir().mkpath(replayDir + "/var");

        QDomNodeList protoNodes = root.elementsByTagName("protocol");
        if (!protoNodes.isEmpty()) {
            QFile protoFile(replayDir + "/var/messages.xml");
            if (protoFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
                QTextStream outT(&protoFile);
                QDomDocument tmpDoc;
                tmpDoc.appendChild(tmpDoc.importNode(protoNodes.at(0), true));
                tmpDoc.save(outT, 2);
            }
        }
    }
    
    void initDictionary() {
        QString pprzHome = qEnvironmentVariable("PAPARAZZI_HOME");
        if (pprzHome.isEmpty()) pprzHome = QDir::currentPath();
        try {
            pprzlink::MessageDictionary dict(pprzHome + "/var/replay/var/messages.xml");
            auto groundDefs = dict.getMsgsForClass("ground");
            for (auto& def : groundDefs) {
                m_groundMsgs.insert(def.getName());
            }
            auto telDefs = dict.getMsgsForClass("telemetry");
            for (auto& def : telDefs) {
                m_telemetryMsgs.insert(def.getName());
            }
            try {
                auto def = dict.getDefinition("WORLD_ENV");
                for (size_t i = 0; i < def.getNbFields(); ++i) {
                    if (def.getField(i).getName() == "time_scale") {
                        break;
                    }
                }
            } catch (...) {}
            }
        } catch (std::exception& e) {
            qWarning() << "Error reading dictionary:" << e.what();
        }
    }

    double m_speed;
    int m_currentIndex;
    bool m_isPlaying;
    bool m_noGui = false;
    QTimer* m_timer;
    QString m_xmlFile;
    QVector<LogEntry> m_log;
    IvyQt* m_bus;
    QString m_ivyBusArg;
    int m_timeScaleIdx = -1;
    
    QSet<QString> m_groundMsgs;
    QSet<QString> m_telemetryMsgs;
};

class PlayWindow : public QMainWindow {
    Q_OBJECT
public:
    PlayWindow(PlayCore* core, QWidget* parent = nullptr) : QMainWindow(parent), m_core(core) {
        setWindowTitle("Replay");
        resize(400, 100);
        
        QWidget* central = new QWidget(this);
        QVBoxLayout* vlayout = new QVBoxLayout(central);
        
        // Toolbar
        QHBoxLayout* tools = new QHBoxLayout();
        QPushButton* btnOpen = new QPushButton("Open Log");
        QPushButton* btnPlay = new QPushButton("Play");
        QPushButton* btnStop = new QPushButton("Stop");
        m_speedBox = new QDoubleSpinBox();
        m_speedBox->setRange(0.1, 100.0);
        m_speedBox->setSingleStep(0.5);
        m_speedBox->setValue(1.0);
        m_speedBox->setPrefix("x ");
        
        tools->addWidget(btnOpen);
        tools->addWidget(btnPlay);
        tools->addWidget(btnStop);
        tools->addWidget(m_speedBox);
        vlayout->addLayout(tools);
        
        // Slider
        m_slider = new QSlider(Qt::Horizontal);
        vlayout->addWidget(m_slider);
        
        setCentralWidget(central);
        
        connect(btnOpen, &QPushButton::clicked, this, &PlayWindow::onOpen);
        connect(btnPlay, &QPushButton::clicked, m_core, &PlayCore::play);
        connect(btnStop, &QPushButton::clicked, m_core, &PlayCore::stop);
        connect(m_speedBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), m_core, &PlayCore::setSpeed);
        connect(m_slider, &QSlider::sliderMoved, this, &PlayWindow::onSliderMoved);
        connect(m_slider, &QSlider::sliderPressed, m_core, &PlayCore::stop);
        
        connect(m_core, &PlayCore::logLoaded, this, &PlayWindow::onLogLoaded);
        connect(m_core, &PlayCore::timeUpdated, this, &PlayWindow::onTimeUpdated);
    }
    
private slots:
    void onOpen() {
        m_core->stop();
        QString fileName;
        {
            StderrBlocker blocker;
            fileName = QFileDialog::getOpenFileName(this, "Open Log", "", "XML Log Files (*.xml);;All Files (*)");
        }
        if (!fileName.isEmpty()) {
            if (m_core->loadLog(fileName)) {
                setWindowTitle(QFileInfo(fileName).fileName());
            }
        }
    }
    
    void onLogLoaded(double minT, double maxT) {
        m_slider->setRange(0, 10000); // 10000 steps resolution
        m_minT = minT;
        m_maxT = maxT;
        m_slider->setValue(0);
    }
    
    void onTimeUpdated(double currentT) {
        if (!m_slider->isSliderDown() && m_maxT > m_minT) {
            double frac = (currentT - m_minT) / (m_maxT - m_minT);
            m_slider->blockSignals(true);
            m_slider->setValue(frac * 10000);
            m_slider->blockSignals(false);
        }
    }
    
    void onSliderMoved(int val) {
        double currentT = m_minT + (val / 10000.0) * (m_maxT - m_minT);
        m_core->setTime(currentT);
    }

private:
    PlayCore* m_core;
    QSlider* m_slider;
    QDoubleSpinBox* m_speedBox;
    double m_minT = 0, m_maxT = 0;
};

int main(int argc, char *argv[]) {
    QCoreApplication::setApplicationVersion("1.0");
    QGuiApplication::setDesktopFileName(QStringLiteral("paparazzi_play"));
    QCoreApplication::setApplicationName(QStringLiteral("Paparazzi replay"));
    qputenv("QT_LOGGING_RULES", "qt.qpa.wayland.textinput=false");

    QApplication app(argc, argv);
    QString iconPath = ":/penguin_icon_rep.png";
    QIcon icon(iconPath);
    installLinuxDesktopIntegration(app.desktopFileName(), "Paparazzi replay", "Player to replay flights", iconPath, "paparazzi-play");
    app.setWindowIcon(icon);

    QCommandLineParser parser;
    parser.setApplicationDescription("Paparazzi Replay");
    parser.addHelpOption();
    parser.addOption(QCommandLineOption(QStringList() << "version", "Displays version information."));
    parser.addOption(QCommandLineOption(QStringList() << "b", "Ivy Bus. Default is 127.255.255.255:2010", "bus", "127.255.255.255:2010"));
    parser.addOption(QCommandLineOption(QStringList() << "d", "Port Default is /dev/ttyUSB0", "port", "/dev/ttyUSB0"));
    parser.addOption(QCommandLineOption(QStringList() << "o", "Output binary messages on serial port"));
    parser.addOption(QCommandLineOption(QStringList() << "s", "Baudrate Default is 9600", "baudrate", "9600"));
    parser.addOption(QCommandLineOption(QStringList() << "shfc", "Enable UART hardware flow control (CTS/RTS)"));
    parser.addOption(QCommandLineOption(QStringList() << "no-gui", "Run without GUI (equivalent to play-nox)"));
    
    parser.addPositionalArgument("log", "Log file to load.", "[log file]");

    parser.process(app);

    if (parser.isSet("version")) {
        parser.showVersion();
        return 0;
    }

    QString ivyBus = parser.value("b");
    if(ivyBus.isEmpty()) ivyBus = "127.255.255.255:2010"; // Default
    bool noGui = parser.isSet("no-gui");
    QStringList args = parser.positionalArguments();
    
    PlayCore core;
    core.setIvyBus(ivyBus);
    core.setNoGui(noGui);
    core.startIvy();

    PlayWindow* window = nullptr;
    if (!noGui) {
        window = new PlayWindow(&core);
        window->show();
    }

    if (!args.isEmpty()) {
        if (core.loadLog(args.first())) {
            double s, e;
            core.getBounds(s, e);
            if (window) {
                window->setWindowTitle(QFileInfo(args.first()).fileName());
            }
            core.play();
        }
    }

    return app.exec();
}

#include "play.moc"
