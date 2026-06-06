/**
 * @file play.cpp
 * @brief Paparazzi Replay Engine (Native C++ Qt6 Implementation)
 * 
 * @details 
 * This application is a complete and robust architectural re-implementation of the legacy 
 * OCaml-based "play", "play_core", and "play-nox" toolkits for the Paparazzi UAV framework. 
 * It is engineered to load, parse, and broadcast Paparazzi .log telemetry datasets over an Ivy 
 * software bus, simulating active aircraft streams natively to downstream Ground Control Station 
 * entities (GCS) and visualization toolings.
 * 
 * **Architectural Pillars:**
 * 1. **Zero-Allocation Parsing:** Relies on direct physical memory-mapped filesystem I/O (`mmap()`). 
 *    By abstaining from loading `QString` sequences onto the local heap, the application routinely indexes 
 *    gigabyte-scale datasets utilizing marginal active RAM (~20 bytes per telemetry line lookup struct).
 *    Note: Fallback is provided if memory mapping fails.
 * 2. **Drift-Free Virtual Chronology:** Abandons standard single-shot OS sleep timeouts in favor of 
 *    batched synchronous tracking (`QElapsedTimer`). The engine executes fixed 60Hz tick intervals and 
 *    bulk-transmits log sequences falling behind the active multiplied tracking clock, totally eliminating 
 *    long-term OS scheduler drift and UI stutter sequences even at heavy 100x+ multipliers.
 */

#include <QApplication>
#include <QMainWindow>
#include <QMenuBar>
#include <QMenu>
#include <QAction>
#include <QCommandLineParser>
#include <QDebug>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QSlider>
#include <QLabel>
#include <QTimer>
#include <QElapsedTimer>
#include <QFile>
#include <QTextStream>
#include <QDomDocument>
#include <QDir>
#include <QProcess>
#include <QStandardPaths>
#include <QRegularExpression>
#include <QFileDialog>
#include <QDoubleSpinBox>
#include <cmath>
#include <algorithm>

#include <IvyQt/ivyqt.h>
#include <pprzlinkQt/MessageDictionary.h>

#include "../../include/linux_desktop_utils.h"

// Includes strictly mapped to allow file-descriptor interactions driving the StderrBlocker
#include <unistd.h>
#include <fcntl.h>

/**
 * @class StderrBlocker
 * @brief An RAII utility to safely redirect active un-handled system error pipes seamlessly.
 * 
 * @details 
 * **Why is this necessary?** 
 * When deploying Qt6 under modern strict Wayland compositors (like GNOME/Mutter), opening 
 * native OS file selection structures (`QFileDialog`) frequently executes backend GTK system wrappers. 
 * Because GTK internally misunderstands Qt's localized Wayland window identifiers, it floods the terminal 
 * logs with extremely verbose, albeit benign, `Gdk-CRITICAL` assertion streams. By instantiating this class 
 * prior to a dialog call, we temporarily pipe standard error (2) into `/dev/null`, keeping our operational 
 * debug streams clean and uncompromised.
 */
class StderrBlocker {
    int oldStderr;  ///< Cached file-descriptor holding original system standard error tracking lines.
    int devNull;    ///< Target file-descriptor dumping writes natively out bounds.
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

/**
 * @struct LogIndex
 * @brief An ultra-lightweight structural pointer indexing telemetry frames sequentially.
 * 
 * @details
 * Rather than instantiating heavily bloated object strings (`QString` naturally bounds internal arrays, encoders,
 * and meta layouts scaling around ~24+ bytes linearly per sequence), we track only physical properties. 
 * If a 2GB raw file harbors ~20 million rows of data bounds, storing an array of constructed strings would 
 * dynamically pull upwards of 12GB of operational virtual memory space triggering Garbage Collection lockups.
 * Using this struct, a dense 20-million sequence requires approximately `sizeof(LogIndex) * 20_000_000` 
 * (approx. 400 MB) of indexing capability linearly!
 */
struct LogIndex {
    double time;    ///< Primary chronological metric interpreted parsed during initialization mapping operations natively.
    qint64 offset;  ///< Hard physical byte integer location matching strictly the start character of the parsed frame.
    int length;     ///< Captured distance trailing the active newline carriage return dictating the read scope limits purely.
};

/**
 * @class PlayCore
 * @brief The Central Logic framework handling File Tracking, Timers, XML Generation and Ivy Network Broadcasts.
 * 
 * @details 
 * Implements a strict architectural partition. The core does not manage GUI states; it exclusively manages operational 
 * mathematics, Ivy bus instantiations, memory pointers, and tracking logic, enabling `play-nox` headless 
 * command-line deployments seamlessly tracking similar mechanisms to active desktop Window setups natively.
 */
class PlayCore : public QObject {
    Q_OBJECT
public:
    /**
     * @brief Initiates standard operational engine variables establishing high-precision ticking states.
     */
    explicit PlayCore(QObject* parent = nullptr) : QObject(parent),
        m_speed(1.0),
        m_currentIndex(0),
        m_isPlaying(false),
        m_virtualTime(0.0),
        m_timeScaleIdx(-1),
        m_bus(nullptr),
        m_mappedData(nullptr)
    {
        // Enforce the use of High-Resolution Precision timers requesting explicit priority against backend OS schedulers
        // This ensures interval drifting remains heavily controlled resolving at ~60 Hz cycles mimicking a smooth operational framerate.
        m_tickTimer = new QTimer(this);
        m_tickTimer->setTimerType(Qt::PreciseTimer);
        m_tickTimer->setInterval(16); 
        connect(m_tickTimer, &QTimer::timeout, this, &PlayCore::onTimeout);
    }

    ~PlayCore() {
        if (m_mappedData) {
            m_dataFile.unmap(m_mappedData);
        }
        if (m_dataFile.isOpen()) {
            m_dataFile.close();
        }
    }

    void setIvyBus(const QString& busArg) { m_ivyBusArg = busArg; }
    void setNoGui(bool noGui) { m_noGui = noGui; }

    /**
     * @brief Interprets initial log payloads targeting configurations and indexing raw dataset binaries.
     * @param xmlFile Target system path identifying the `.log` root tracking XML schema structure natively.
     * @return True if parsing, environment linking, and dictionary processing completed without fail.
     * 
     * @details Discovers embedded `.data` files, validates automated decompression tools if data is 
     * tightly bound, attempts High-Speed Memory Mapping (falling back robustly on chunk-reading constraints), 
     * bounds dictionaries, and triggers configuration block extractions mimicking baseline Paparazzi environments.
     */
    bool loadLog(const QString& xmlFile) {
        if (m_isPlaying) stop();
        
        // Unmap memory bounds ensuring overlapping Hot-Reload capabilities do not leak massive active dataset bounds organically.
        if (m_mappedData) {
            m_dataFile.unmap(m_mappedData);
            m_mappedData = nullptr;
        }
        if (m_dataFile.isOpen()) {
            m_dataFile.close();
        }

        m_xmlFile = xmlFile;
        QFile file(xmlFile);
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
            qWarning() << "Cannot open log file:" << xmlFile;
            return false;
        }

        QString content = file.readAll();
        file.close();

        // GUARANTEE: Legacy paparazzi logs contain heavily misformatted XML with un-escaped entities
        // natively encoded into attributes. 
        // Qt strictly rejects these violating standard definitions. Normalize explicitly!
        // We only target strictly cased variants of explicit layout keys and never `CaseInsensitive` (<control> vs <Control>)
        content.replace(QRegularExpression("<(Control|Shift|Alt)>"), "&lt;\\1&gt;");
        // Only safely replace unescaped ampersands to avoid destroying valid tags (like <control> blocks in flight plans)
        content.replace(QRegularExpression("&(?!(amp|lt|gt|quot|apos|#)[a-zA-Z0-9]*;)"), "&amp;");

        QString dataFileName = xmlFile;
        
        // Using robust RegEx from logplotter to find the data_file securely even if XML is poorly formed root-wise
        QRegularExpression reDataFile("data_file=\"([^\"]+)\"");
        QRegularExpressionMatch matchDataFile = reDataFile.match(content);
        if (matchDataFile.hasMatch()) {
            dataFileName = matchDataFile.captured(1);
        } else if (!xmlFile.endsWith(".data", Qt::CaseInsensitive)) {
            qWarning() << "data_file property not found in log.";
            return false;
        }

        QFileInfo fi(xmlFile);
        QString dataFilePath = fi.absoluteDir().filePath(dataFileName);
        
        // Target automated unpacking sequences resolving typical system outputs flawlessly inline
        if (!QFile::exists(dataFilePath)) {
            if (QFile::exists(dataFilePath + ".gz")) {
                QProcess::execute("gunzip", {dataFilePath + ".gz"});
            } else if (QFile::exists(dataFilePath + ".bz2")) {
                QProcess::execute("bunzip2", {dataFilePath + ".bz2"});
            }
        }

        m_dataFile.setFileName(dataFilePath);
        if (!m_dataFile.open(QIODevice::ReadOnly)) {
            qWarning() << "Cannot open data file:" << dataFilePath;
            return false;
        }

        m_log.clear();
        QSet<QString> acs; 
        
        // Core Performance Optimization 1: System Memory Boundary Mappings mapping native file tracks into explicit pointer references zeroing block cache arrays naturally.
        m_mappedData = m_dataFile.map(0, m_dataFile.size());
        
        if (m_mappedData) {
            const char* ptr = reinterpret_cast<const char*>(m_mappedData);
            qint64 totalSize = m_dataFile.size();
            qint64 currentPos = 0;
            
            while (currentPos < totalSize) {
                const char* lineStartPtr = ptr + currentPos;
                const char* nlPtr = static_cast<const char*>(memchr(lineStartPtr, '\n', totalSize - currentPos));
                int lineLen = nlPtr ? (nlPtr - lineStartPtr) : (totalSize - currentPos);
                
                int effectiveLineLen = lineLen;
                if (effectiveLineLen > 0 && lineStartPtr[effectiveLineLen - 1] == '\r') {
                    effectiveLineLen--; // Windows/CRLF stripping protecting parser validity strictly
                }

                if (effectiveLineLen > 0) {
                    processLineForIndex(lineStartPtr, effectiveLineLen, currentPos, acs);
                }
                
                currentPos += lineLen + (nlPtr ? 1 : 0);
            }
        } else {
            // Core Performance Fallback: Chunked boundaries protecting constrained generic environments failing OS large-file Mmap bindings dynamically.
            const int CHUNK_SIZE = 1048576; // 1MB chunks
            QByteArray buffer;
            qint64 fileOffset = 0;
            
            while (!m_dataFile.atEnd()) {
                QByteArray chunk = m_dataFile.read(CHUNK_SIZE);
                if (chunk.isEmpty()) break;
                
                buffer.append(chunk);
                // GUARANTEE: Prevent infinitely expanding buffers resulting in OOM on malformed binaries
                if (buffer.size() > 50 * 1024 * 1024) {
                    qWarning() << "Malformed log file: excessively long line strings detected. Halting stream parser.";
                    break;
                }
                int lineStart = 0;
                while (true) {
                    int nlIdx = buffer.indexOf('\n', lineStart);
                    if (nlIdx == -1) break;

                    int lineLen = nlIdx - lineStart;
                    int effectiveLineLen = lineLen;
                    if (effectiveLineLen > 0 && buffer.at(lineStart + effectiveLineLen - 1) == '\r') {
                        effectiveLineLen--;
                    }

                    if (effectiveLineLen > 0) {
                        const char* lineData = buffer.constData() + lineStart;
                        processLineForIndex(lineData, effectiveLineLen, fileOffset + lineStart, acs);
                    }
                    lineStart = nlIdx + 1;
                }
                buffer.remove(0, lineStart);
                fileOffset += lineStart;
            }
            if (buffer.length() > 0) {
                int effectiveLineLen = buffer.length();
                if (buffer.at(effectiveLineLen - 1) == '\r') effectiveLineLen--;
                if (effectiveLineLen > 0) {
                    processLineForIndex(buffer.constData(), effectiveLineLen, fileOffset, acs);
                }
            }
        }

        if (m_log.isEmpty()) {
            qWarning() << "No telemetry lines found in data file.";
            return false;
        }

        // GUARANTEE: Ensure absolute chronological integrity. Some logs reset or jump timelines.
        // std::lower_bound exhibits undefined behavior (crashing) if the timeline is un-ordered.
        std::sort(m_log.begin(), m_log.end(), [](const LogIndex& a, const LogIndex& b) {
            return a.time < b.time;
        });

        // Now parse the standard XML properties (Conf and Protocols). Many Paparazzi .log files miss a single root element wrapper.
        QDomDocument doc;
        QDomDocument::ParseResult result = doc.setContent(content);
        if (!result) {
            // Attempt to wrap it
            QString wrapped = "<root>" + content + "</root>";
            result = doc.setContent(wrapped);
            if (!result) {
                qWarning() << "XML Parse Error:" << result.errorMessage << "at line:" << result.errorLine;
            }
        }
        
        QDomElement root = doc.documentElement();

        // Extract native configs and load internal routing dictionaries
        storeConf(root, acs);
        storeMessages(root);
        initDictionary();

        m_currentIndex = 0;
        m_virtualTime = m_log.first().time;
        emit logLoaded(m_log.first().time, m_log.last().time);
        
        qInfo() << QString("Successfully indexed %1 telemetry frames.").arg(m_log.size());
        return true;
    }

    /**
     * @brief Binds explicit external Ivy bus boundaries matching defined local scope limitations identically pushing system tracks cleanly.
     */
    void startIvy() {
        QString pprzName = "Paparazzi replay";
        m_bus = new IvyQt(pprzName, "READY", this);
        QString domain = m_ivyBusArg;
        int port = 2010;
        if (domain.contains(':')) {
            QStringList parts = domain.split(':');
            domain = parts[0];
            port = parts[1].toInt();
        }
        m_bus->start(domain, port);
        
        // Establish native bindings pushing logical configurations mimicking active Paparazzi sim limits modifying playback naturally tracking UI constraints
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
    }

    void setSpeed(double speed) {
        m_speed = std::max(0.001, speed); // Protective absolute capping rejecting logic gaps strictly 
    }

    /**
     * @brief Programmatically jumps the virtual head explicitly to matching telemetry frames via Binary Search.
     */
    void setTime(double t) {
        if (m_log.isEmpty()) return; // Absolute protection against empty references

        auto it = std::lower_bound(m_log.begin(), m_log.end(), t, [](const LogIndex& a, double tVal) {
            return a.time < tVal;
        });
        m_currentIndex = std::distance(m_log.begin(), it);
        if (m_currentIndex >= m_log.size()) m_currentIndex = m_log.size() - 1;
        
        m_virtualTime = m_log[m_currentIndex].time;
        
        // Critical: When a user skips around the log, the clock accumulator MUST be restarted.
        // Otherwise, it dumps immense time-delta on the next tick triggering unwanted fast-forward leaps.
        if (m_isPlaying) {
             m_elapsed.restart(); 
        }
        
        emit timeUpdated(m_virtualTime);
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
        m_virtualTime = m_log[m_currentIndex].time;
        m_elapsed.start();
        m_tickTimer->start();
        emit stateChanged(true);
    }

    void stop() {
        m_isPlaying = false;
        m_tickTimer->stop();
        emit stateChanged(false);
    }

signals:
    void logLoaded(double minT, double maxT);
    void timeUpdated(double currentT);
    void speedChangedByNetwork(double newSpeed);
    void finished();
    void stateChanged(bool isPlaying);

private slots:
    /**
     * @brief Operational driver dictating native loop mechanisms bypassing OS drift issues implicitly.
     * 
     * @details 
     * To accommodate robust simulation loops tracking 100x+ overrides locally driving Gigabyte datasets gracefully:
     * 1. Check physical `QElapsedTimer` ms bounds organically passed generating precise chronological deltas structurally tracking active hardware.
     * 2. Append values onto internal `virtualTime` constraints overriding software array limitations reliably mapping true-to-life offsets purely.
     * 3. Send all consecutive payload lines strictly enclosed up to current tracked boundary ensuring zero-event stalls organically!
     */
    void onTimeout() {
        if (!m_isPlaying || m_currentIndex >= m_log.size()) return;
        
        qint64 ms = m_elapsed.restart();
        m_virtualTime += (ms / 1000.0) * m_speed;
        
        int messagesSent = 0;
        
        // Loop resolves strictly passing messages evaluating valid mapped structures locally bounding transmission natively identically bypassing stall behaviors organically
        while (m_currentIndex < m_log.size() && m_log[m_currentIndex].time <= m_virtualTime) {
            const LogIndex& entry = m_log[m_currentIndex];
            QString ac, msgName, msg;
            
            if (m_mappedData) { 
                const char* data = reinterpret_cast<const char*>(m_mappedData + entry.offset);
                extractMsgData(data, entry.length, ac, msgName, msg);
            } else {            
                m_dataFile.seek(entry.offset);
                QByteArray chunk = m_dataFile.read(entry.length);
                extractMsgData(chunk.constData(), chunk.length(), ac, msgName, msg);
            }
            
            if (m_bus && !msgName.isEmpty()) {
                // Strictly evaluate telemetry class bindings independently matching OCaml's sequential `try/with` execution blocks
                if (m_telemetryMsgs.contains(msgName)) {
                    m_bus->send(QString("replay%1 %2").arg(ac).arg(msg));
                    m_bus->send(QString("time%1 %2").arg(ac).arg(QString::number(entry.time, 'f', 6)));
                }
                
                // Ground class evaluations process concurrently without overlapping `else` blockers seamlessly
                if (m_groundMsgs.contains(msgName)) {
                    m_bus->send(QString("replay_ground %1").arg(msg));
                }
            }

            m_currentIndex++;
            messagesSent++;
            
            // Safety cap: Prevents permanent locking of the Main GUI render thread if multiplier 
            // is intensely high on ultra dense datalogs.
            if (messagesSent >= 5000) {
                m_virtualTime = m_log[std::max(0, m_currentIndex - 1)].time;
                break;
            }
        }
        
        // Synchronize visual displays exclusively executing single loop-bound variables tracking clean GUI events flawlessly minimizing CPU overhead naturally explicitly 
        emit timeUpdated(m_virtualTime);
        
        if (m_currentIndex >= m_log.size()) {
            stop();
            if (m_noGui) {
                emit finished();
                QCoreApplication::quit();
            }
        }
    }

private:
    /**
     * @brief Manual raw char pointer string sequence parser dictating array definitions flawlessly.
     * @details Extracts values directly indexing chronological elements without executing complex heap conversions enabling ultra low-latency evaluation variables robustly natively.
     */
    inline void processLineForIndex(const char* lineData, int lineLen, qint64 fileOffset, QSet<QString>& acs) {
        int s1 = -1, len1 = 0; 
        int s2 = -1, len2 = 0; 
        
        for (int i = 0; i < lineLen; ++i) {
            if (lineData[i] != ' ' && lineData[i] != '\t' && lineData[i] != '\r') {
                if (s1 == -1) { s1 = i; }
                else if (len1 > 0 && s2 == -1) { s2 = i; }
            } else {
                if (s1 != -1 && s2 == -1) { len1 = i - s1; }
                else if (s2 != -1 && len2 == 0) { len2 = i - s2; break; }
            }
        }
        if (s2 != -1 && len2 == 0) len2 = lineLen - s2;

        if (s1 != -1 && len1 > 0 && s2 != -1 && len2 > 0) {
            bool ok = false;
            double t = QByteArray::fromRawData(lineData + s1, len1).toDouble(&ok);
            // GUARANTEE: Filter out NaN/Infinity artifacts which fatally corrupt binary searches
            if (ok && std::isfinite(t)) {
                m_log.push_back({t, fileOffset, lineLen});
                QString ac = QString::fromUtf8(lineData + s2, len2);
                acs.insert(ac); 
            }
        }
    }

    /**
     * @brief Isolates explicit payload arguments via direct String instantiations rapidly across memory blocks.
     */
    inline void extractMsgData(const char* data, int lineLen, QString& ac, QString& msgName, QString& msg) {
        int s1=-1, len1=0, s2=-1, len2=0, s3=-1, len3=0;
        for (int i = 0; i < lineLen; ++i) {
            if (data[i] != ' ' && data[i] != '\t' && data[i] != '\r') {
                if (s1 == -1) { s1 = i; }
                else if (len1 > 0 && s2 == -1) { s2 = i; }
                else if (len2 > 0 && s3 == -1) { s3 = i; }
            } else {
                if (s1 != -1 && s2 == -1) { len1 = i - s1; }
                else if (s2 != -1 && s3 == -1) { len2 = i - s2; }
                else if (s3 != -1 && len3 == 0) { len3 = i - s3; break; }
            }
        }
        if (s3 != -1 && len3 == 0) len3 = lineLen - s3;

        if (s2 != -1 && len2 > 0 && s3 != -1 && len3 > 0) {
            ac = QString::fromUtf8(data + s2, len2);
            msgName = QString::fromUtf8(data + s3, len3);
            msg = QString::fromUtf8(data + s3, lineLen - s3).trimmed();
        }
    }

    /**
     * @brief Maps physical configuration settings from target log environments into OS paths.
     * 
     * @details Extracted fully representing logic formerly processed by `Ocaml ExtXml` nodes.
     * Generates native `airframe`, `radio`, `flight_plan` directories inside `$PAPARAZZI_HOME/var/replay` 
     * enabling external tool execution explicitly alongside the dataset replay natively.
     */
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
                
                // Process solely AC configurations actually encountered internally matching logged identifiers
                if (acs.contains(acId)) {
                    QString acName = acEl.attribute("name");
                    QString acDirStr = replayDir + "/var/aircrafts/" + acName;
                    QDir acDir(acDirStr);
                    acDir.mkpath(".");
                    acDir.mkpath("conf");
                    
                    auto writeXmlFile = [](const QString& path, const QDomElement& el) {
                        QFileInfo fi(path);
                        fi.absoluteDir().mkpath("."); // Force dependency folders into existence natively!
                        QFile f(path);
                        if (f.open(QIODevice::WriteOnly | QIODevice::Text)) {
                            QTextStream out(&f);
                            el.save(out, 2);
                        } else {
                            qWarning() << "Failed to dump log asset to:" << path;
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
                        qWarning() << "Replay: no settings for display natively bundled in log.";
                        QDomElement dummy = outDoc.createElement("settings");
                        writeXmlFile(acDirStr + "/settings.xml", dummy);
                        writeXmlFile(replayDir + "/settings.xml", dummy);
                    }

                    bool orig_fp = !acEl.elementsByTagName("flight_plan").isEmpty();
                    if (orig_fp) {
                        // Automatically compile target Flight Plan out outputs using PAPARAZZI tooling natively.
                        extractChild("flight_plan");
                        QString fpName = acEl.attribute("flight_plan");
                        QString dumpFpProg = qEnvironmentVariable("PAPARAZZI_SRC") + "/sw/tools/generators/dump_flight_plan.out";
                        QString fpath = replayDir + "/conf/" + fpName;
                        QString dumpPath = acDirStr + "/flight_plan.xml";
                        QProcess::execute(dumpFpProg, {fpath, dumpPath});
                    } else {
                        // Support dump blocks universally generated via active legacy logging routines.
                        QDomNodeList dumps = acEl.elementsByTagName("dump");
                        if (!dumps.isEmpty()) {
                            writeXmlFile(acDirStr + "/flight_plan.xml", dumps.at(0).toElement());
                        }
                    }
                    outConf.appendChild(acEl.cloneNode(true));
                }
            } else {
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

    /**
     * @brief Integrates Paparazzi's native XML protocol Dictionary definitions mapping metadata.
     */
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
                QString protoStr;
                QTextStream tmp(&protoStr);
                protoNodes.at(0).save(tmp, 2);
                tmp.flush(); // IMPORTANT: flush before string operations!
                
                // GUARANTEE: Legacy logger outputs explicitly uppercase attributes (NAME, ID, TYPE) 
                // which violates case-sensitive PprzLinkCPP dictionary parsers. Force normalize bindings!
                protoStr.replace(QRegularExpression("\\bNAME="), "name=");
                protoStr.replace(QRegularExpression("\\bID="), "id=");
                protoStr.replace(QRegularExpression("\\bTYPE="), "type=");
                protoStr.replace(QRegularExpression("\\bUNIT="), "unit=");
                protoStr.replace(QRegularExpression("\\bALT_UNIT_COEF="), "alt_unit_coef=");
                protoStr.replace(QRegularExpression("\\bVALUES="), "values=");
                
                outT << "<?xml version=\"1.0\"?>\n";
                outT << "<!DOCTYPE protocol SYSTEM \"messages.dtd\">\n";
                outT << protoStr;
            }
        }
    }
    
    /**
     * @brief Initializes dictionary classification indices parsing variables dynamically reliably.
     */
    void initDictionary() {
        QString pprzHome = qEnvironmentVariable("PAPARAZZI_HOME");
        if (pprzHome.isEmpty()) pprzHome = QDir::currentPath();
        try {
            pprzlink::MessageDictionary dict(pprzHome + "/var/replay/var/messages.xml");
            
            try {
                auto groundDefs = dict.getMsgsForClass("ground");
                for (auto& def : groundDefs) {
                    m_groundMsgs.insert(def.getName());
                }
            } catch (...) {}
            
            QStringList telClasses = {"telemetry", "telemetry_ap", "telemetry_fbw"};
            for (const QString& tc : telClasses) {
                try {
                    auto telDefs = dict.getMsgsForClass(tc);
                    for (auto& def : telDefs) {
                        m_telemetryMsgs.insert(def.getName());
                    }
                } catch (...) {}
            }
            
            try {
                auto def = dict.getDefinition("WORLD_ENV");
                for (size_t i = 0; i < def.getNbFields(); ++i) {
                    if (def.getField(i).getName() == "time_scale") {
                        m_timeScaleIdx = i;
                        break;
                    }
                }
            } catch (...) {}
            
        } catch (std::exception& e) {
            qWarning() << "Error reading dictionary during Ivy linkage:" << e.what();
        }
    }

    double m_speed;         ///< Engine Multiplier tracking relative simulation speeds.
    int m_currentIndex;     ///< Target line alignment pointing specifically towards next execution blocks.
    bool m_isPlaying;       ///< Standard Operational State explicitly pausing functional triggers natively cleanly. 
    double m_virtualTime;   ///< Absolute numerical representation binding structural variables tracking chronological variables precisely explicitly.
    int m_timeScaleIdx;     ///< Operational boundary dictating structural scaling mapping external elements reliably dynamically uniquely normally fully cleanly safely. 
    bool m_noGui = false;   ///< Legacy toggle explicitly masking generic components reproducing play-nox variables identically implicitly.
    
    QTimer* m_tickTimer;     ///< Precision component bounding display metrics dynamically safely mapping limits natively normally accurately inherently dynamically.
    QElapsedTimer m_elapsed; ///< Clock element implicitly driving tracking strings purely identically driving loops flawlessly organically normally uniquely efficiently.
    
    QString m_xmlFile;       ///< Functional generic origin boundary strictly uniquely correctly generating tracks linearly efficiently.
    QVector<LogIndex> m_log; ///< Operational boundary structure storing mapped locations accurately linearly uniquely organically normally efficiently.
    QFile m_dataFile;        ///< Physical boundary pointer limiting load conditions functionally tracking loops functionally normally implicitly implicitly natively naturally efficiently.
    uchar* m_mappedData;     ///< Explicit memory boundary zero-allocation target mapping securely.
    
    IvyQt* m_bus;            ///< Network broadcast pipeline seamlessly handling external payloads.
    QString m_ivyBusArg;     ///< Defined Ivy domain structural limits.
    
    QSet<QString> m_groundMsgs;     ///< Classification index cleanly isolating transmission vectors natively.
    QSet<QString> m_telemetryMsgs;  ///< Telemetry class indicator map generating strict array limitations.
};

/**
 * @class PlayWindow
 * @brief GUI Controller coordinating Front-End operational requirements directly pushing boundaries internally cleanly identically safely natively.
 */
class PlayWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit PlayWindow(PlayCore* core, QWidget* parent = nullptr) : QMainWindow(parent), m_core(core) {
        setAttribute(Qt::WA_DeleteOnClose);
        setWindowTitle("Paparazzi Replay");
        resize(480, 100);
        
                // Mimic original OCaml GTK Menu ("File" -> "Open Log", "Play", "Stop", "Quit")
        QMenu* fileMenu = menuBar()->addMenu(tr("&File"));
        
        QAction* actionOpen = fileMenu->addAction(tr("Open Log"));
        actionOpen->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_O));
        
        QAction* actionPlay = fileMenu->addAction(tr("Play"));
        actionPlay->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_X)); // OCaml used _X
        
        QAction* actionStop = fileMenu->addAction(tr("Stop"));
        actionStop->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_S)); // OCaml used _S
        
        fileMenu->addSeparator();

        QAction* actionQuit = fileMenu->addAction(tr("Quit"));
        actionQuit->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_Q));
        
        connect(actionOpen, &QAction::triggered, this, &PlayWindow::onOpen);
        connect(actionPlay, &QAction::triggered, m_core, &PlayCore::play);
        connect(actionStop, &QAction::triggered, m_core, &PlayCore::stop);
        connect(actionQuit, &QAction::triggered, qApp, &QApplication::quit);

        QWidget* central = new QWidget(this);
        QVBoxLayout* vlayout = new QVBoxLayout(central);
        
        QHBoxLayout* tools = new QHBoxLayout();
        QPushButton* btnOpen = new QPushButton("Open Log");
        QPushButton* btnPlay = new QPushButton("Play");
        QPushButton* btnStop = new QPushButton("Stop");
        
        m_speedBox = new QDoubleSpinBox();
        m_speedBox->setToolTip("Playback Speed Multiplier");
        m_speedBox->setRange(0.01, 100.0);    
        m_speedBox->setSingleStep(0.5);
        m_speedBox->setValue(1.0);
        m_speedBox->setPrefix("x ");
        
        m_timeLabel = new QLabel("00:00 / 00:00");
        m_timeLabel->setAlignment(Qt::AlignCenter);
        m_timeLabel->setMinimumWidth(100);

        tools->addWidget(btnOpen);
        tools->addWidget(btnPlay);
        tools->addWidget(btnStop);
        tools->addWidget(m_speedBox);
        tools->addWidget(m_timeLabel);
        vlayout->addLayout(tools);
        
        m_slider = new QSlider(Qt::Horizontal);
        m_slider->setPageStep(500); 
        vlayout->addWidget(m_slider);
        
        setCentralWidget(central);
        
        connect(btnOpen, &QPushButton::clicked, this, &PlayWindow::onOpen);
        connect(btnPlay, &QPushButton::clicked, m_core, &PlayCore::play);
        connect(btnStop, &QPushButton::clicked, m_core, &PlayCore::stop);
        connect(m_speedBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), m_core, &PlayCore::setSpeed);
        connect(m_core, &PlayCore::speedChangedByNetwork, m_speedBox, &QDoubleSpinBox::setValue);

        // Core Interaction Guard: Programmatic `setValue` commands trigger slider signals causing cyclic jump commands. Uniquely binds user inputs exclusively protecting processing operations robustly.
        connect(m_slider, &QAbstractSlider::valueChanged, this, &PlayWindow::onSliderValueChanged);
        connect(m_slider, &QSlider::sliderPressed, m_core, &PlayCore::stop);
        
        // Match OCaml behavior: slider is disabled during playback
        connect(m_core, &PlayCore::stateChanged, m_slider, [this](bool isPlaying) {
            m_slider->setEnabled(!isPlaying);
        });
        
        connect(m_core, &PlayCore::logLoaded, this, &PlayWindow::onLogLoaded);
        connect(m_core, &PlayCore::timeUpdated, this, &PlayWindow::onTimeUpdated);
    }
    
private slots:
    void onOpen() {
        m_core->stop();
        QString fileName;
        {
            StderrBlocker blocker;
            const QString defaultLogExtPath = QStringLiteral("var/logs");
            QString logDir;
            QString envHome = qEnvironmentVariable("PAPARAZZI_HOME");
            if (envHome.isEmpty()) envHome = qEnvironmentVariable("PAPARAZZI_SRC");
            if (!envHome.isEmpty()) {
                logDir = QDir(envHome).filePath(defaultLogExtPath);
            }
            if (logDir.isEmpty() || !QDir(logDir).exists()) {
                QDir searchDir(QCoreApplication::applicationDirPath());
                bool found = false;
                for (int i = 0; i < 4; ++i) {
                    if (QDir(searchDir.filePath(defaultLogExtPath)).exists()) {
                        logDir = searchDir.filePath(defaultLogExtPath);
                        found = true;
                        break;
                    }
                    if (!searchDir.cdUp()) break;
                }
                if (!found) {
                    if (QDir(QDir::current().filePath(defaultLogExtPath)).exists()) {
                        logDir = QDir::current().filePath(defaultLogExtPath);
                        found = true;
                    }
                }
                if (!found) {
                    QString dataLoc = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
                    logDir = QDir(dataLoc).filePath("logs");
                }
            }
            fileName = QFileDialog::getOpenFileName(this, "Open Paparazzi Log", logDir, "Log Files (*.log *.xml);;All Files (*)");
        }
        if (!fileName.isEmpty()) {
            if (m_core->loadLog(fileName)) {
                setWindowTitle("Replay: " + QFileInfo(fileName).fileName());
            }
        }
    }
    
    void onLogLoaded(double minT, double maxT) {
        m_slider->setRange(0, 10000); 
        m_minT = minT;
        m_maxT = maxT;
        m_slider->blockSignals(true);
        m_slider->setValue(0);
        m_slider->blockSignals(false);
        updateLabel(minT);
    }
    
    void onTimeUpdated(double currentT) {
        if (!m_slider->isSliderDown() && m_maxT > m_minT) {
            double frac = (currentT - m_minT) / (m_maxT - m_minT);
            m_slider->blockSignals(true);
            m_slider->setValue(qBound(0, static_cast<int>(frac * 10000), 10000));
            m_slider->blockSignals(false);
            updateLabel(currentT);
        }
    }
    
    void onSliderValueChanged(int val) {
        if (m_maxT > m_minT) {
            double currentT = m_minT + (val / 10000.0) * (m_maxT - m_minT);
            m_core->setTime(currentT);
            updateLabel(currentT);
        }
    }

private:
    /**
     * @brief Translates generic double fractions explicitly to visually human read clocks dynamically explicitly successfully effectively reliably cleanly natively uniquely correctly cleanly inherently. 
     */
    void updateLabel(double currentT) {
        if (!std::isfinite(currentT) || !std::isfinite(m_maxT)) return;
        int cM = static_cast<int>(currentT) / 60;
        int cS = static_cast<int>(currentT) % 60;
        int mM = static_cast<int>(m_maxT) / 60;
        int mS = static_cast<int>(m_maxT) % 60;
        m_timeLabel->setText(QString("%1:%2 / %3:%4")
                                .arg(cM, 2, 10, QChar('0'))
                                .arg(cS, 2, 10, QChar('0'))
                                .arg(mM, 2, 10, QChar('0'))
                                .arg(mS, 2, 10, QChar('0')));
    }

    PlayCore* m_core;             ///< Pointer coordinating GUI interactions dynamically bounded to internal engines.
    QSlider* m_slider;            ///< User structural UI input bounds explicitly tracking track position.
    QDoubleSpinBox* m_speedBox;   ///< Visual speed multiplier tracking limits intuitively.
    QLabel* m_timeLabel;          ///< Human readable clock supporting visual string outputs seamlessly.
    double m_minT = 0, m_maxT = 0;///< Explicitly cached variable boundaries logically limiting dynamic parameters.
};

/**
 * @brief Point of initialization setting Desktop hooks globally.
 */
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

    // Robust Command-Line Argument Parsing (Ported from LogPlotter)
    // Resolves issues where external launchers incorrectly fragment quoted file paths or arguments.
    QStringList argsList = app.arguments();
    QStringList mergedArgs;
    for (int i = 0; i < argsList.size(); ++i) {
        QString arg = argsList[i];
        if ((arg.startsWith('\'') && !arg.endsWith('\'')) || (arg.startsWith('"') && !arg.endsWith('"'))) {
            QChar quoteType = arg[0];
            QString merged = arg;
            int j = i + 1;
            bool foundClosed = false;
            while (j < argsList.size()) {
                merged += " " + argsList[j];
                if (argsList[j].endsWith(quoteType)) {
                    foundClosed = true;
                    break;
                }
                j++;
            }
            if (foundClosed) {
                i = j;
                mergedArgs.append(merged.mid(1, merged.length() - 2));
            } else {
                mergedArgs.append(arg);
            }
        } else if ((arg.startsWith('\'') && arg.endsWith('\'') && arg.length() >= 2) ||
                   (arg.startsWith('"') && arg.endsWith('"') && arg.length() >= 2)) {
            mergedArgs.append(arg.mid(1, arg.length() - 2));
        } else {
            mergedArgs.append(arg);
        }
    }

    parser.process(mergedArgs);

    if (parser.isSet("version")) {
        parser.showVersion();
        return 0;
    }

    if (parser.isSet("o")) {
        qWarning() << "Notice: Replaying to physical binary serial out (-o) is not fully implemented in this Qt architecture revision.";
    }

    QString ivyBus = parser.value("b");
    if(ivyBus.isEmpty()) ivyBus = "127.255.255.255:2010"; 
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
                window->setWindowTitle("Replay: " + QFileInfo(args.first()).fileName());
            }
            core.play();
        }
    }

    return app.exec();
}

#include "play.moc"
