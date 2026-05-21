#include <QApplication>
#include <QDebug>
#include <QDomDocument>
#include <QFile>
#include <QDrag>
#include <QHBoxLayout>
#include <QIcon>
#include <QLabel>
#include <QListWidget>
#include <QListWidgetItem>
#include <QMainWindow>
#include <QMap>
#include <QMimeData>
#include <QScrollBar>
#include <QSizePolicy>
#include <QMouseEvent>
#include <QStyle>
#include <QPushButton>
#include <QSplitter>
#include <QStackedWidget>
#include <QTabWidget>
#include <QTimer>
#include <QTime>
#include <QVBoxLayout>
#include <QVariant>
#include <QStandardPaths>
#include <QDir>
#include <QTextStream>
#include <QProcess>
#include <sstream>
#include <variant>
#include "pprzlinkQt/IvyQtLink.h"
#include "pprzlinkQt/Message.h"
#include "pprzlinkQt/MessageDictionary.h"
#include "pprzlinkQt/MessageDefinition.h"
#include "pprzlinkQt/MessageField.h"
#include "pprzlinkQt/FieldValue.h"

struct MsgTracker {
    QWidget* pageWidget;
    QLabel* timeLabel;
    QWidget* timeBox;
};

class SenderTab : public QWidget {
    Q_OBJECT
public:
    explicit SenderTab(const QString& senderName, const QString& className, pprzlink::MessageDictionary* dict, QWidget* parent = nullptr);
    void handleMessage(const pprzlink::Message& msg);

private slots:
    void updateTimers();

private:
    QString m_senderName;
    QString m_className;
    pprzlink::MessageDictionary* m_dict;

    QListWidget* m_listWidget;
    QStackedWidget* m_stackedWidget;

    int m_listWidgetWidth = 100;
    void updateListWidgetWidth(int contentWidth);

    QMap<QString, MsgTracker> m_msgTrackers;
    QMap<QString, QMap<QString, QLabel*>> m_fieldLabels;
};

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    QTabWidget* m_classTabWidget;
    QLabel* m_waitingLabel;
    QMap<QString, SenderTab*> m_senderTabs;
    pprzlink::MessageDictionary* m_dict = nullptr;
    pprzlink::IvyQtLink* m_link = nullptr;

    void setupDictionaryAndLink();
};


static QMap<QString, QMap<QString, QMap<QString, QString>>> s_unitCoefs;
static QMap<QString, QMap<QString, QMap<QString, QString>>> s_unitNames;
static constexpr int GREEN_DECAY_RATE_MS = 200;//TODO: make it based on message rate set in telemetry file.

static void loadUnitCoefs(const QString& xmlPath) {
    QFile file(xmlPath);
    if (!file.open(QIODevice::ReadOnly)) {
        qWarning() << "Could not open" << xmlPath << "to parse unit coefs";
        return;
    }
    QDomDocument doc("mydocument");
    if (!doc.setContent(&file)) {
        file.close();
        return;
    }
    file.close();
    
    QDomElement docElem = doc.documentElement();
    QDomNode n = docElem.firstChild();
    while(!n.isNull()) {
        QDomElement e = n.toElement(); 
        if(!e.isNull() && e.tagName() == "msg_class") {
            QString className = e.attribute("name");
            QDomNode m = e.firstChild();
            while(!m.isNull()) {
                QDomElement me = m.toElement();
                if(!me.isNull() && me.tagName() == "message") {
                    QString msgName = me.attribute("name");
                    QDomNode f = me.firstChild();
                    while(!f.isNull()) {
                        QDomElement fe = f.toElement();
                        if(!fe.isNull() && fe.tagName() == "field") {
                            QString fieldName = fe.attribute("name");
                            QString coef = fe.attribute("alt_unit_coef", "1.");
                            s_unitCoefs[className][msgName][fieldName] = coef;
                            s_unitNames[className][msgName][fieldName] = fe.attribute("alt_unit", fe.attribute("unit", ""));
                        }
                        f = f.nextSibling();
                    }
                }
                m = m.nextSibling();
            }
        }
        n = n.nextSibling();
    }
}

static QString senderIdToString(const std::variant<QString, uint8_t> &senderV)
{
    if (std::holds_alternative<QString>(senderV)) {
        return std::get<QString>(senderV).trimmed();
    }
    return QString::number(static_cast<int>(std::get<uint8_t>(senderV)));
}

static QString safeFieldName(const QString& fieldName, int index)
{
    if (!fieldName.isEmpty()) {
        return fieldName;
    }
    return QStringLiteral("field_%1").arg(index);
}

static QString safeMessageName(const QString& msgName)
{
    if (!msgName.isEmpty()) {
        return msgName;
    }
    return QStringLiteral("<unknown message>");
}

class DraggableButton : public QPushButton {
public:
    DraggableButton(const QString& text, const QString& payload, QWidget* parent = nullptr)
        : QPushButton(text, parent), m_payload(payload) {
        setToolTip("Drag-and-drop field on:\n\t- Real-Time Plotter to plot a curve\n\t- GCS map to display as a papget");
    }

protected:
    void mousePressEvent(QMouseEvent *event) override {
        QPushButton::mousePressEvent(event);
        if (event->button() == Qt::LeftButton) {
            m_dragStartPos = event->pos();
        }
    }
    
    void mouseMoveEvent(QMouseEvent *event) override {
        if (!(event->buttons() & Qt::LeftButton))
            return;
        if ((event->pos() - m_dragStartPos).manhattanLength() < QApplication::startDragDistance())
            return;

        QDrag *drag = new QDrag(this);
        QMimeData *mimeData = new QMimeData;
        mimeData->setText(m_payload);
        drag->setMimeData(mimeData);
        drag->exec(Qt::CopyAction | Qt::MoveAction);
    }
private:
    QPoint m_dragStartPos;
    QString m_payload;
};

SenderTab::SenderTab(const QString& senderName, const QString& className, pprzlink::MessageDictionary* dict, QWidget* parent)
    : QWidget(parent), m_senderName(senderName), m_className(className), m_dict(dict) {
    auto layout = new QHBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    
    QSplitter* splitter = new QSplitter(Qt::Horizontal, this);
    
    m_listWidget = new QListWidget(this);
    m_listWidget->setFrameShape(QFrame::NoFrame);
    m_listWidget->setAttribute(Qt::WA_MacShowFocusRect, false);
    m_listWidget->setSelectionMode(QAbstractItemView::SingleSelection);
    m_listWidget->setMinimumWidth(100);
    //m_listWidget->setMaximumWidth(100);
    m_listWidget->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);
    m_listWidget->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    
    m_stackedWidget = new QStackedWidget(this);
    m_stackedWidget->setMinimumWidth(100);
    m_stackedWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    
    splitter->addWidget(m_listWidget);
    splitter->addWidget(m_stackedWidget);
    splitter->setHandleWidth(0);
    splitter->setStretchFactor(0, 0);
    splitter->setStretchFactor(1, 1);
    splitter->setCollapsible(0, false);
    splitter->setCollapsible(1, false);
    
    layout->addWidget(splitter);
    
    connect(m_listWidget, &QListWidget::currentRowChanged, m_stackedWidget, &QStackedWidget::setCurrentIndex);

    QTimer* globalTimer = new QTimer(this);
    connect(globalTimer, &QTimer::timeout, this, &SenderTab::updateTimers);
    globalTimer->start(50);
}

void SenderTab::updateTimers() {
    for(auto& t : m_msgTrackers) {
        if (!t.timeLabel || !t.timeBox) {
            continue;
        }

        QVariant lastUpdate = t.timeLabel->property("lastUpdate");
        if (!lastUpdate.isValid() || !lastUpdate.canConvert<QTime>()) {
            t.timeLabel->setProperty("lastUpdate", QTime::currentTime());
            continue;
        }

        int msecs = lastUpdate.toTime().msecsTo(QTime::currentTime());
        if (msecs < 0 || msecs > 99999999) {
            msecs = 0; // Just in case of midnight wrap
        }

        if (msecs > GREEN_DECAY_RATE_MS) {
            t.timeBox->setStyleSheet(".QWidget { background-color: #000000; border-radius: 0px; }\nQLabel { color: #fff; font-weight: bold; }");
        }

        if (msecs > 1999) {
            t.timeLabel->setText(QString::number(msecs / 1000));
        } else {
            t.timeLabel->setText("");
        }
    }
}

void SenderTab::updateListWidgetWidth(int contentWidth) {
    const int scrollbarWidth = m_listWidget->verticalScrollBar()->isVisible()
        ? m_listWidget->verticalScrollBar()->sizeHint().width()
        : m_listWidget->style()->pixelMetric(QStyle::PM_ScrollBarExtent);
    const int extraPadding = 20; // extra buffer for item margins and layout spacing
    const int desiredWidth = qMax(100, contentWidth + scrollbarWidth + extraPadding);

    if (desiredWidth <= m_listWidgetWidth)
        return;

    m_listWidgetWidth = desiredWidth;
    m_listWidget->setMinimumWidth(m_listWidgetWidth);
    m_listWidget->setMaximumWidth(m_listWidgetWidth);
}

void SenderTab::handleMessage(const pprzlink::Message& msg) {
    QString msgName = safeMessageName(msg.getDefinition().getName());
    
    if (!m_msgTrackers.contains(msgName)) {
        QWidget* page = new QWidget(this);
        QVBoxLayout* vlayout = new QVBoxLayout(page);
        
        // Item in list widget
        QListWidgetItem* item = new QListWidgetItem();
        
        // Custom widget for list item
        QWidget* itemWidget = new QWidget(m_listWidget);
        itemWidget->setStyleSheet("background: transparent;");
        itemWidget->setStyleSheet("background: transparent;");
        QHBoxLayout* itemLayout = new QHBoxLayout(itemWidget);
        itemLayout->setContentsMargins(4, 2, 4, 2);
        
        QLabel* nameLabel = new QLabel(msgName);
        nameLabel->setAlignment(Qt::AlignCenter);
        
        QLabel* timeLabel = new QLabel("");
        timeLabel->setMinimumWidth(40);
        timeLabel->setAlignment(Qt::AlignCenter);
        timeLabel->setProperty("lastUpdate", QTime::currentTime());
        
        QWidget* timeBox = new QWidget();
        QHBoxLayout* tBoxL = new QHBoxLayout(timeBox);
        tBoxL->setContentsMargins(2, 2, 2, 2);
        tBoxL->addWidget(timeLabel);
        timeBox->setStyleSheet(".QWidget { background-color: #000000; border-radius: 0px; }\nQLabel { color: #fff; font-weight: bold; }");
        
        itemLayout->addStretch();
        itemLayout->addWidget(nameLabel);
        itemLayout->addStretch();
        itemLayout->addWidget(timeBox);
        
        itemWidget->adjustSize();
        item->setSizeHint(itemWidget->sizeHint());
        item->setData(Qt::UserRole, msgName);
        updateListWidgetWidth(itemWidget->sizeHint().width());
        
        int insertRow = 0;
        for (; insertRow < m_listWidget->count(); ++insertRow) {
            const QString existingName = m_listWidget->item(insertRow)->data(Qt::UserRole).toString();
            if (existingName > msgName) break;
        }
        m_listWidget->insertItem(insertRow, item);
        m_listWidget->setItemWidget(item, itemWidget);
        m_stackedWidget->insertWidget(insertRow, page);
        
        MsgTracker tracker;
        tracker.pageWidget = page;
        tracker.timeLabel = timeLabel;
        tracker.timeBox = timeBox;
        m_msgTrackers[msgName] = tracker;
        
        // Fields for the right page
        const auto& def = msg.getDefinition();
        for (int i = 0; i < (int)def.getNbFields(); ++i) {
            const auto& field = def.getField(i);
            QHBoxLayout* hlayout = new QHBoxLayout();
            
            QString fieldName = safeFieldName(field.getName(), i);
            QString typeName = field.getType().toString();
            
            QString coef = s_unitCoefs[m_className][msgName][fieldName];
            if (coef.isEmpty()) coef = "1.";
            QString unit = s_unitNames[m_className][msgName][fieldName];
            
            QString btnText = typeName + " " + fieldName + (unit.isEmpty() ? "" : ": (" + unit + ")");
            QString payload = m_senderName + ":" + m_className + ":" + msgName + ":" + fieldName + ":" + coef;
            
            DraggableButton* btn = new DraggableButton(btnText, payload, page);
            QLabel* valLabel = new QLabel("XXXX", page);
            
            hlayout->addWidget(btn);
            hlayout->addWidget(valLabel);
            hlayout->addStretch();
            
            vlayout->addLayout(hlayout);
            m_fieldLabels[msgName][fieldName] = valLabel;
        }
        vlayout->addStretch();
    }
    
    // Update values
    if (!m_msgTrackers.contains(msgName)) {
        qWarning() << "Received message with unknown name" << msgName;
        return;
    }

    MsgTracker& tracker = m_msgTrackers[msgName];
    if (!tracker.timeLabel || !tracker.timeBox) {
        qWarning() << "Invalid tracker for message" << msgName;
        return;
    }

    tracker.timeLabel->setProperty("lastUpdate", QTime::currentTime());
    tracker.timeLabel->setText("");
    
    // Briefly flash green background
    tracker.timeBox->setStyleSheet(".QWidget { background-color: #22ff22; border-radius: 0px; }\nQLabel { color: #000; font-weight: bold; }");
    
    const auto& def = msg.getDefinition();
    for (int i = 0; i < (int)def.getNbFields(); ++i) {
        const auto& field = def.getField(i);
        if (field.getName().isEmpty()) {
            continue;
        }

        try {
            auto rv = msg.getRawValue(i);
            rv.setOutputInt8AsInt(true);
            std::stringstream ss;
            ss << rv;
            QString s = QString::fromStdString(ss.str());
            
            auto fieldMapIt = m_fieldLabels.find(msgName);
            if (fieldMapIt == m_fieldLabels.end()) {
                continue;
            }
            const auto fieldMap = fieldMapIt.value();
            if (!fieldMap.contains(field.getName())) {
                continue;
            }
            QLabel* lbl = fieldMap.value(field.getName());
            if (lbl) {
                lbl->setText(s);
            }
        } catch(const std::exception &ex) {
            qWarning() << "Failed to read field value for" << msgName << "field" << field.getName() << ":" << ex.what();
        } catch(...) {
            qWarning() << "Unknown error while updating field value for" << msgName;
        }
    }
}

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
    setWindowTitle("Messages");
    resize(300, 400);// TODO: Dynamically resize based available screen size, with sensible limits. Note that it DOES resize based on content requirement after startup and messages come in.

    QWidget* cntral = new QWidget(this);
    QVBoxLayout* layout = new QVBoxLayout(cntral);
    layout->setContentsMargins(0, 0, 0, 0);

    m_waitingLabel = new QLabel("Initializing telemetry...", this);
    m_waitingLabel->setAlignment(Qt::AlignCenter);

    m_classTabWidget = new QTabWidget(this);
    m_classTabWidget->hide();

    layout->addWidget(m_waitingLabel);
    layout->addWidget(m_classTabWidget);
    setCentralWidget(cntral);
    
    setupDictionaryAndLink();
}

MainWindow::~MainWindow() {
    if (m_link) {
        m_link->stop();
        delete m_link;
    }
    if (m_dict) delete m_dict;
}

void MainWindow::setupDictionaryAndLink() {
    QString phome = qgetenv("PAPARAZZI_HOME");
    if (phome.isEmpty()) phome = QString("/home/%1/paparazzi").arg(qgetenv("USER"));
    QString xmlPath = phome + "/var/messages.xml";

    if (!QFile::exists(xmlPath)) {
        m_waitingLabel->setText(tr("Missing messages.xml at %1").arg(xmlPath));
        qWarning() << "Missing messages.xml at" << xmlPath;
        return;
    }

    loadUnitCoefs(xmlPath);

    try {
        m_dict = new pprzlink::MessageDictionary(xmlPath);
        if (!m_dict) {
            throw std::runtime_error("Failed to allocate MessageDictionary");
        }
        m_link = new pprzlink::IvyQtLink(*m_dict, "messages_qt", this);
        if (!m_link) {
            throw std::runtime_error("Failed to allocate IvyQtLink");
        }
        m_waitingLabel->setText(tr("Starting Ivy bus..."));
        connect(m_link, &pprzlink::IvyQtLink::serverConnected, this, [this]() {
            if (m_waitingLabel && m_waitingLabel->isVisible()) {
                m_waitingLabel->setText(tr("Connected to Ivy bus just fine,\nbut still waiting for telemetry data..."));
            }
        });
        m_link->start("127.255.255.255:2010");
        if (m_waitingLabel && m_waitingLabel->isVisible()) {
            m_waitingLabel->setText(tr("Waiting for telemetry data..."));
        }
    } catch (const std::exception &ex) {
        qWarning() << "Failed to initialize messaging:" << ex.what();
        m_waitingLabel->setText(tr("Telemetry initialization failed"));
        delete m_link;
        m_link = nullptr;
        delete m_dict;
        m_dict = nullptr;
        return;
    } catch (...) {
        qWarning() << "Failed to initialize messaging due to unknown error.";
        m_waitingLabel->setText(tr("Telemetry initialization failed"));
        delete m_link;
        m_link = nullptr;
        delete m_dict;
        m_dict = nullptr;
        return;
    }

    if (!m_dict) {
        m_waitingLabel->setText(tr("Telemetry dictionary unavailable"));
        return;
    }

    QString className = "telemetry";
    const auto msgs = m_dict->getMsgsForClass(className);
    if (msgs.empty()) {
        m_waitingLabel->setText(tr("No telemetry message definitions found for %1").arg(className));
        qWarning() << "No telemetry message definitions found for" << className;
        return;
    }

    for (const auto& def : msgs) {
        if (!m_link) {
            break;
        }
        m_link->BindMessage(def, this, [=](QString sender, pprzlink::Message msg) {
            if (m_waitingLabel && m_waitingLabel->isVisible()) {
                m_waitingLabel->hide();
                if (m_classTabWidget) {
                    m_classTabWidget->show();
                }
            }

            QString sId = sender.trimmed();
            if (sId.isEmpty()) {
                const auto& senderV = msg.getSenderId();
                sId = senderIdToString(senderV);
            }
            if (sId.isEmpty()) {
                sId = QStringLiteral("ground");
            }

            QString tabName = className + ":" + sId;
            if (!m_senderTabs.contains(tabName)) {
                SenderTab* tab = new SenderTab(sId, className, m_dict, this);
                m_classTabWidget->addTab(tab, tabName);
                m_senderTabs[tabName] = tab;
            }

            if (m_senderTabs.contains(tabName)) {
                m_senderTabs[tabName]->handleMessage(msg);
            }
        });
    }
}

int main(int argc, char *argv[])
{
    // Force GTK3 platform theme which natively supports Ubuntu Adwaita dark/light
    // qputenv("QT_QPA_PLATFORMTHEME", "gtk3");//QT5 and fallback for QT6<6.8, also seems to work fine with QT6.8+ as gtk3 is not strictly required for dark mode support on newer Qt versions.
    
    QApplication app(argc, argv);
    app.setApplicationName(QStringLiteral("paparazzi_messages"));
    //app.setApplicationDisplayName(QStringLiteral("Paparazzi Messages"));//How much is too much ;)
    app.setDesktopFileName(QStringLiteral("paparazzi-messages"));

#if defined(Q_OS_LINUX)
    // Dynamically install desktop integration files so GNOME/Wayland can pick them up dynamically
    QString exePath = QCoreApplication::arguments().at(0);
    if (!exePath.contains("/")) {
        exePath = QStandardPaths::findExecutable(exePath);
    } else {
        exePath = QDir::cleanPath(QDir().absoluteFilePath(exePath));
    }

    QString userName = qgetenv("USER");
    if (!userName.isEmpty() && exePath.startsWith("/home/" + userName + "/")) {
        // Substitute /home/user/ with ~/ internally wrapped in a bash exec so it's fully portable
        // and doesn't pollute the .desktop file with hardcoded sensitive user names.
        // E.g. bash -c "exec ~/paparazzi/.../messages"
        exePath.replace(0, ("/home/" + userName).length(), "~");
        //Yes, backslashes are there to escape the doublequotes indeed
        exePath = "bash -c \"exec " + exePath + "\"";//overkill? Just exec with ~ directly, it seems to work fine in .desktop files and is more transparent.
        //exePath = "\"" + exePath + "\"";
    }

    QString appsLocation = QStandardPaths::writableLocation(QStandardPaths::ApplicationsLocation);
    if (!appsLocation.isEmpty()) {
        QDir().mkpath(appsLocation);
        QString desktopFilePath = appsLocation + "/paparazzi-messages.desktop";
        QFile dfile(desktopFilePath);
        if (dfile.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QTextStream out(&dfile);
            out << "[Desktop Entry]\n"
                << "Version=1.0\n"
                << "Type=Application\n"
                << "Name=Paparazzi Messages\n"
                << "Comment=View and inspect telemetry messages in the Paparazzi ground segment\n"
                << "Exec=" << exePath << "\n"
                << "Icon=paparazzi-messages\n"
                << "Terminal=false\n"
                << "Categories=Development;Education;Viewer;Science;Robotics;\n"
                << "StartupNotify=true\n"
                << "StartupWMClass=paparazzi_messages\n";
            dfile.close();
        }

        QString iconDir = QStandardPaths::writableLocation(QStandardPaths::GenericDataLocation) + "/icons/hicolor/128x128/apps";
        QDir().mkpath(iconDir);
        QString iconFilePath = iconDir + "/paparazzi-messages.png";
        if (QFile::exists(iconFilePath)) {
            QFile::remove(iconFilePath);
        }
        QFile::copy(":/penguin_icon_msg.png", iconFilePath);
        
        // Let the system catch up using Qt's native cross-platform process API
        // Although it would be more efficient to call the underlying update-desktop-database and gtk-update-icon-cache
        // functions directly via a native platform API, this approach is more maintainable and portable, 
        // and the performance impact should be negligible since it's only done once at startup.
        // YEAH: if you want to get fancy, feel free to improve
        QProcess::startDetached("update-desktop-database", QStringList() << appsLocation);
        QString hicolorDir = QStandardPaths::writableLocation(QStandardPaths::GenericDataLocation) + "/icons/hicolor";
        QProcess::startDetached("gtk-update-icon-cache", QStringList() << "-f" << "-t" << hicolorDir);
    }
#endif

    QIcon icon(QStringLiteral(":/penguin_icon_msg.png"));
    app.setWindowIcon(icon);

    MainWindow w;
    w.setWindowIcon(icon);
    w.show();
    return app.exec();
}

#include "messages.moc"

