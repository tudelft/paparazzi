/**
 * @file messages.cpp
 * @brief Paparazzi Telemetry Messages Viewer.
 * 
 * This file implements the Qt-based UI for monitoring and inspecting Ivy telemetry messages. It dynamically parses the unit and coefficient geometries to reflect real-time telemetry variables.
 */

#include <QApplication>
#include <QCommandLineParser>
#include <QCommandLineOption>
#include <QMimeData>
#include <QDrag>
#include <QLabel>
#include <QListWidgetItem>
#include <QMainWindow>
#include <QScrollBar>
#include <QMouseEvent>
#include <QPushButton>
#include <QStackedWidget>
#include <QVBoxLayout>
#include <QInputDialog>
#include <QRegularExpression>

#include "pprzlinkQt/IvyQtLink.h"
#include "../../include/linux_desktop_utils.h"

/**
 * @brief Represents the MessagesConfig struct.
 * @details This struct encapsulates the primary logic and UI structures required 
 * for MessagesConfig operations, ensuring robust and memory-safe management within the telemetry pipeline.
 */
struct MessagesConfig {
    QString ivyBus;
    QStringList classes;
    bool timestamp;
    bool force;
    QString geometry;
    MessagesConfig() : timestamp(false), force(false) {}
};

/**
 * @brief Represents the MsgTracker struct.
 * @details This struct encapsulates the primary logic and UI structures required 
 * for MsgTracker operations, ensuring robust and memory-safe management within the telemetry pipeline.
 */
struct MsgTracker {
    QLabel* timeLabel = nullptr;
    QWidget* timeBox = nullptr;
    QVector<QLabel*> fieldLabels;
    QVector<class DraggableButton*> fieldButtons;
    qint64 lastUpdateMs = 0;
    bool isGreen = false;
    int lastSecs = -1;
};

/**
 * @brief Represents the SenderTab class.
 * @details This class encapsulates the primary logic and UI structures required 
 * for SenderTab operations, ensuring robust and memory-safe management within the telemetry pipeline.
 */
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

    QHash<QString, MsgTracker> m_msgTrackers;
};

/**
 * @brief Represents the MessagesWindow class.
 * @details This class encapsulates the primary logic and UI structures required 
 * for MessagesWindow operations, ensuring robust and memory-safe management within the telemetry pipeline.
 */
class MessagesWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MessagesWindow(const MessagesConfig& config, QWidget *parent = nullptr);
    ~MessagesWindow();

private:
    MessagesConfig m_config;
    QTabWidget* m_classTabWidget;
    QLabel* m_waitingLabel;
    QHash<QString, SenderTab*> m_senderTabs;
    pprzlink::MessageDictionary* m_dict = nullptr;
    pprzlink::IvyQtLink* m_link = nullptr;

    void setupDictionaryAndLink();
};


/**
 * @brief Represents the FieldInfo struct.
 * @details This struct encapsulates the primary logic and UI structures required 
 * for FieldInfo operations, ensuring robust and memory-safe management within the telemetry pipeline.
 */
struct FieldInfo {
    QString coef;
    QString unit;
};
static QHash<QString, QHash<QString, QHash<QString, FieldInfo>>> s_fieldInfos;
static constexpr int GREEN_DECAY_RATE_MS = 200;

#include <QXmlStreamReader>
static void loadUnitCoefs(const QString& xmlPath) {
    QFile file(xmlPath);
    if (!file.open(QIODevice::ReadOnly)) {
        qWarning() << "Could not open" << xmlPath << "to parse unit coefs";
        return;
    }
    QXmlStreamReader xml(&file);
    QString currentClass, currentMessage;
    while (!xml.atEnd() && !xml.hasError()) {
        QXmlStreamReader::TokenType token = xml.readNext();
        if (token == QXmlStreamReader::StartElement) {
            auto name = xml.name();
            if (name == QLatin1String("msg_class")) {
                currentClass = xml.attributes().value(QLatin1String("name")).toString();
            } else if (name == QLatin1String("message")) {
                currentMessage = xml.attributes().value(QLatin1String("name")).toString();
            } else if (name == QLatin1String("field")) {
                auto attrs = xml.attributes();
                QString fieldName = attrs.value(QLatin1String("name")).toString();
                QString coef = attrs.value(QLatin1String("alt_unit_coef")).toString();
                if (coef.isEmpty()) coef = QStringLiteral("1.");
                QString unit = attrs.value(QLatin1String("alt_unit")).toString();
                if (unit.isEmpty()) unit = attrs.value(QLatin1String("unit")).toString();
                
                s_fieldInfos[currentClass][currentMessage][fieldName] = {coef, unit};
            }
        }
        else if (token == QXmlStreamReader::EndElement) {
            auto name = xml.name();
            if (name == QLatin1String("msg_class")) currentClass.clear();
            else if (name == QLatin1String("message")) currentMessage.clear();
        }
    }
    if (xml.hasError()) {
        qWarning() << "XML error in" << xmlPath << ":" << xml.errorString();
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



/**
 * @brief Represents the DraggableButton class.
 * @details This class encapsulates the primary logic and UI structures required 
 * for DraggableButton operations, ensuring robust and memory-safe management within the telemetry pipeline.
 */
class DraggableButton : public QPushButton {
public:
    DraggableButton(const QString& text, const QString& senderName, const QString& className, const QString& msgName, const QString& fieldName, const QString& coef, bool isArray, int arraySize, QWidget* parent = nullptr)
        : QPushButton(text, parent), m_senderName(senderName), m_className(className), m_msgName(msgName), m_fieldName(fieldName), m_coef(coef), m_isArray(isArray), m_arraySize(arraySize) {
        setToolTip("Drag-and-drop field on:\n\t- Real-Time Plotter to plot a curve\n\t- GCS map to display as a papget");
    }

    void setDynamicArraySize(int sz) {
        if (m_isArray) m_arraySize = sz;
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
        QString delayedFilePath;
        if (m_isArray) {
            delayedFilePath = QString("/tmp/pprz_dnd_%1_%2.txt").arg(QCoreApplication::applicationPid()).arg(QDateTime::currentMSecsSinceEpoch());
            QFile::remove(delayedFilePath); // ensure clear
            mimeData->setText("delayed_array:" + delayedFilePath);
        } else {
            QString payload = m_senderName + ":" + m_className + ":" + m_msgName + ":" + m_fieldName + ":" + m_coef;
            mimeData->setText(payload);
        }
        drag->setMimeData(mimeData);

        Qt::DropAction action = drag->exec(Qt::CopyAction | Qt::MoveAction);
        drag->deleteLater(); // Prevent QDrag memory leak on repeated drops

        if (m_isArray && action != Qt::IgnoreAction) {
            QString defaultRange = (m_arraySize > 0) ? QString("0-%1").arg(m_arraySize - 1) : "0";
            
            QInputDialog dialog(nullptr);
            dialog.setWindowFlags(Qt::Window | Qt::WindowStaysOnTopHint);
            dialog.setWindowTitle("Index of value to plot");
            dialog.setLabelText("Index or range in the array?");
            dialog.setTextValue(defaultRange);

            QStringList parts;
            if (dialog.exec() == QDialog::Accepted) {
                QString text = dialog.textValue();
                QList<int> indices;
                QStringList tokens = text.split(QRegularExpression("[,;\\s]+"), Qt::SkipEmptyParts);
                for (const QString& token : tokens) {
                    if (token.contains("-")) {
                        QStringList range = token.split("-");
                        if (range.size() == 2) {
                            int start = range[0].toInt();
                            int end = range[1].toInt();
                            if (start <= end) {
                                for (int i = start; i <= end; ++i) {
                                    if (i >= 0 && (m_arraySize == 0 || i < m_arraySize)) indices.append(i);
                                }
                            }
                        }
                    } else {
                        int i = token.toInt();
                        if (i >= 0 && (m_arraySize == 0 || i < m_arraySize)) indices.append(i);
                    }
                }
                for (int n : indices) {
                    parts << QString("%1:%2:%3:%4[%5]:%6").arg(m_senderName).arg(m_className).arg(m_msgName).arg(m_fieldName).arg(n).arg(m_coef);
                }
                
                QFile f(delayedFilePath);
                if (f.open(QIODevice::WriteOnly | QIODevice::Text)) {
                    QTextStream out(&f);
                    out << parts.join('\n');
                    f.close();
                }
            } else {
                // write empty to unblock receiver
                QFile f(delayedFilePath);
                if (f.open(QIODevice::WriteOnly | QIODevice::Text)) {
                    QTextStream out(&f);
                    out << "";
                    f.close();
                }
            }
        }
    }
private:
    QPoint m_dragStartPos;
    QString m_senderName;
    QString m_className;
    QString m_msgName;
    QString m_fieldName;
    QString m_coef;
    bool m_isArray;
    int m_arraySize;
};

SenderTab::SenderTab(const QString& senderName, const QString& className, pprzlink::MessageDictionary* dict, QWidget* parent)
    : QWidget(parent), m_senderName(senderName), m_className(className), m_dict(dict) {
    auto layout = new QHBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);
    
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
    
    layout->addWidget(m_listWidget);
    layout->addWidget(m_stackedWidget, 1);
    
    connect(m_listWidget, &QListWidget::currentRowChanged, m_stackedWidget, &QStackedWidget::setCurrentIndex);

    QTimer* globalTimer = new QTimer(this);
    connect(globalTimer, &QTimer::timeout, this, &SenderTab::updateTimers);
    globalTimer->start(50);
}

void SenderTab::updateTimers() {
    qint64 now = QDateTime::currentMSecsSinceEpoch();
    for(auto& t : m_msgTrackers) {
        if (!t.timeLabel || !t.timeBox || t.lastUpdateMs == 0) {
            continue;
        }

        qint64 msecs = now - t.lastUpdateMs;
        if (msecs < 0) {
            msecs = 0;
        }

        if (t.isGreen && msecs > GREEN_DECAY_RATE_MS) {
            t.timeBox->setStyleSheet(".QWidget { background-color: #000000; border-radius: 0px; }\nQLabel { color: #fff; font-weight: bold; }");
            t.isGreen = false;
            t.lastSecs = -1; // force text update
        }

        if (msecs > 1999) {
            int secs = static_cast<int>(msecs / 1000);
            if (secs != t.lastSecs) {
                t.timeLabel->setText(QString::number(secs));
                t.lastSecs = secs;
            }
        } else if (t.lastSecs != 0) {
            t.timeLabel->setText("");
            t.lastSecs = 0;
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
        QHBoxLayout* itemLayout = new QHBoxLayout(itemWidget);
        itemLayout->setContentsMargins(4, 2, 4, 2);
        
        QLabel* nameLabel = new QLabel(msgName);
        nameLabel->setAlignment(Qt::AlignCenter);
        
        QLabel* timeLabel = new QLabel("");
        timeLabel->setMinimumWidth(40);
        timeLabel->setAlignment(Qt::AlignCenter);
        
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
            auto widgetItem = m_listWidget->item(insertRow);
            if (widgetItem) {
                const QString existingName = widgetItem->data(Qt::UserRole).toString();
                if (existingName > msgName) break;
            }
        }
        m_listWidget->insertItem(insertRow, item);
        m_listWidget->setItemWidget(item, itemWidget);
        m_stackedWidget->insertWidget(insertRow, page);
        
        MsgTracker tracker;
        tracker.timeLabel = timeLabel;
        tracker.timeBox = timeBox;
        
        // Fields for the right page
        const auto& def = msg.getDefinition();
        tracker.fieldLabels.resize(def.getNbFields());
        tracker.fieldButtons.resize(def.getNbFields());
        for (int i = 0; i < (int)def.getNbFields(); ++i) {
            const auto& field = def.getField(i);
            QHBoxLayout* hlayout = new QHBoxLayout();
            
            QString fieldName = safeFieldName(field.getName(), i);
            QString typeName = field.getType().toString();
            
            QString coef = QStringLiteral("1.");
            QString unit;
            auto classIt = s_fieldInfos.constFind(m_className);
            if (classIt != s_fieldInfos.constEnd()) {
                auto msgIt = classIt->constFind(msgName);
                if (msgIt != classIt->constEnd()) {
                    auto fieldIt = msgIt->constFind(fieldName);
                    if (fieldIt != msgIt->constEnd()) {
                        coef = fieldIt->coef;
                        unit = fieldIt->unit;
                    }
                }
            }
            
            QString btnText = typeName + " " + fieldName + (unit.isEmpty() ? "" : ": (" + unit + ")");
            
            DraggableButton* btn = new DraggableButton(btnText, m_senderName, m_className, msgName, fieldName, coef, field.getType().isArray(), field.getType().getArraySize(), page);
            QLabel* valLabel = new QLabel("XXXX", page);
            
            hlayout->addWidget(btn);
            hlayout->addWidget(valLabel);
            hlayout->addStretch();
            
            vlayout->addLayout(hlayout);
            tracker.fieldLabels[i] = valLabel;
            tracker.fieldButtons[i] = btn;
        }
        vlayout->addStretch();
        m_msgTrackers.insert(msgName, tracker);
    }
    
    // Update values
    auto it = m_msgTrackers.find(msgName);
    if (it == m_msgTrackers.end()) {
        qWarning() << "Received message with unknown name" << msgName;
        return;
    }

    MsgTracker& tracker = it.value();
    if (!tracker.timeLabel || !tracker.timeBox) {
        qWarning() << "Invalid tracker for message" << msgName;
        return;
    }

    tracker.lastUpdateMs = QDateTime::currentMSecsSinceEpoch();
    if (tracker.lastSecs != 0) {
        tracker.timeLabel->setText("");
        tracker.lastSecs = 0;
    }
    
    // Briefly flash green background
    if (!tracker.isGreen) {
        tracker.timeBox->setStyleSheet(".QWidget { background-color: #22ff22; border-radius: 0px; }\nQLabel { color: #000; font-weight: bold; }");
        tracker.isGreen = true;
    }
    
    const auto& def = msg.getDefinition();
    for (int i = 0; i < (int)def.getNbFields(); ++i) {
        if (i >= tracker.fieldLabels.size() || !tracker.fieldLabels[i]) {
            continue;
        }

        try {
            auto rv = msg.getRawValue(i);
            const auto& type = def.getField(i).getType();
            if (!type.isArray()) {
                switch (type.getBaseType()) {
                    case pprzlink::BaseType::CHAR: { char v; rv.getValue(v); tracker.fieldLabels[i]->setText(QString::number(static_cast<int>(v))); } break;
                    case pprzlink::BaseType::INT8: { int8_t v; rv.getValue(v); tracker.fieldLabels[i]->setText(QString::number(v)); } break;
                    case pprzlink::BaseType::INT16: { int16_t v; rv.getValue(v); tracker.fieldLabels[i]->setText(QString::number(v)); } break;
                    case pprzlink::BaseType::INT32: { int32_t v; rv.getValue(v); tracker.fieldLabels[i]->setText(QString::number(v)); } break;
                    case pprzlink::BaseType::UINT8: { uint8_t v; rv.getValue(v); tracker.fieldLabels[i]->setText(QString::number(v)); } break;
                    case pprzlink::BaseType::UINT16: { uint16_t v; rv.getValue(v); tracker.fieldLabels[i]->setText(QString::number(v)); } break;
                    case pprzlink::BaseType::UINT32: { uint32_t v; rv.getValue(v); tracker.fieldLabels[i]->setText(QString::number(v)); } break;
                    case pprzlink::BaseType::FLOAT: { float v; rv.getValue(v); tracker.fieldLabels[i]->setText(QString::number(v, 'g', 6)); } break;
                    case pprzlink::BaseType::DOUBLE: { double v; rv.getValue(v); tracker.fieldLabels[i]->setText(QString::number(v, 'g', 6)); } break;
                    case pprzlink::BaseType::STRING: { QString v; rv.getValue(v); tracker.fieldLabels[i]->setText(v); } break;
                    default: {
                        rv.setOutputInt8AsInt(true);
                        std::stringstream ss;
                        ss << rv;
                        tracker.fieldLabels[i]->setText(QString::fromStdString(ss.str()));
                    } break;
                }
            } else {
                try {
                    int dynSize = 0;
                    switch (type.getBaseType()) {
                        case pprzlink::BaseType::CHAR: { std::vector<char> v; rv.getValue(v); dynSize = v.size(); } break;
                        case pprzlink::BaseType::INT8: { std::vector<int8_t> v; rv.getValue(v); dynSize = v.size(); } break;
                        case pprzlink::BaseType::INT16: { std::vector<int16_t> v; rv.getValue(v); dynSize = v.size(); } break;
                        case pprzlink::BaseType::INT32: { std::vector<int32_t> v; rv.getValue(v); dynSize = v.size(); } break;
                        case pprzlink::BaseType::UINT8: { std::vector<uint8_t> v; rv.getValue(v); dynSize = v.size(); } break;
                        case pprzlink::BaseType::UINT16: { std::vector<uint16_t> v; rv.getValue(v); dynSize = v.size(); } break;
                        case pprzlink::BaseType::UINT32: { std::vector<uint32_t> v; rv.getValue(v); dynSize = v.size(); } break;
                        case pprzlink::BaseType::FLOAT: { std::vector<float> v; rv.getValue(v); dynSize = v.size(); } break;
                        case pprzlink::BaseType::DOUBLE: { std::vector<double> v; rv.getValue(v); dynSize = v.size(); } break;
                        default: break;
                    }
                    if (dynSize > 0) {
                        tracker.fieldButtons[i]->setDynamicArraySize(dynSize);
                    }
                } catch (...) {}
                rv.setOutputInt8AsInt(true);
                std::stringstream ss;
                ss << rv;
                tracker.fieldLabels[i]->setText(QString::fromStdString(ss.str()));
            }
        } catch(const std::exception &ex) {
            qWarning() << "Failed to read field value for" << msgName << "field index" << i << ":" << ex.what();
        } catch(...) {
            qWarning() << "Unknown error while updating field value for" << msgName;
        }
    }
}

MessagesWindow::MessagesWindow(const MessagesConfig& config, QWidget *parent) : QMainWindow(parent), m_config(config) {
    setWindowTitle("Messages");

    if (!m_config.geometry.isEmpty()) {
        QRegularExpression re("^(\\d+)x(\\d+)(?:\\+[-]?(\\d+)\\+[-]?(\\d+))?$");
        QRegularExpressionMatch match = re.match(m_config.geometry);
        if (match.hasMatch()) {
            int w = match.captured(1).toInt();
            int h = match.captured(2).toInt();
            resize(w, h);
            if (!match.captured(3).isEmpty() && !match.captured(4).isEmpty()) {
                int x = match.captured(3).toInt();
                int y = match.captured(4).toInt();
                move(x, y);
            }
        } else {
            resize(400, 400);
        }
    } else {
        resize(400, 400);
    }

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

MessagesWindow::~MessagesWindow() {
    if (m_link) {
        m_link->stop();
        delete m_link;
    }
    if (m_dict) delete m_dict;
}

void MessagesWindow::setupDictionaryAndLink() {
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
        m_waitingLabel->setText(tr("Waiting for Ivy bus..."));
        connect(m_link, &pprzlink::IvyQtLink::serverConnected, this, [this]() {
            if (m_waitingLabel && m_waitingLabel->isVisible()) {
                m_waitingLabel->setText(tr("Connected to Ivy bus just fine,\nbut still waiting for telemetry data..."));
            }
        });
        m_link->start(m_config.ivyBus.isEmpty() ? "127.255.255.255:2010" : m_config.ivyBus);
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

    struct ClassFilter {
        QString className;
        QString sender;
    };
    QList<ClassFilter> filters;
    QStringList classArgs = m_config.classes;
    if (classArgs.isEmpty()) {
        classArgs.append("telemetry:*");
    }

    for (const QString& c : classArgs) {
        QStringList parts = c.split(':');
        ClassFilter f;
        f.className = parts[0];
        f.sender = (parts.size() > 1) ? parts[1] : "*";
        filters.append(f);
    }

    for (const ClassFilter& filter : filters) {
        QString className = filter.className;
        const auto msgs = m_dict->getMsgsForClass(className);
        if (msgs.empty()) {
            qWarning() << "No message definitions found for class" << className;
            continue;
        }

        for (const auto& def : msgs) {
            if (!m_link) break;
            
            // To emulate OCaml's non-force behavior on the local side, we can filter out creating tabs
            // until ALIVE is seen, or we just bind everything as IvyQtLink doesn't support sender-specific binding anyway.
            
            m_link->BindMessage(def, this, [this, className, filter, def](const QString& sender, const pprzlink::Message& msg) {
                QString sId = sender.trimmed();
                if (sId.isEmpty()) {
                    const auto& senderV = msg.getSenderId();
                    sId = senderIdToString(senderV);
                }
                if (sId.isEmpty()) {
                    sId = QStringLiteral("ground");
                }

                if (filter.sender != "*" && filter.sender != sId) {
                    return; // Skip messages from unrequested sender
                }

                if (!m_config.force && className == "telemetry" && filter.sender == "*" && def.getName() != "ALIVE") {
                    // Only start tracking a sender if we've seen ALIVE, or if we have force enabled
                    QString tabName = className + ":" + sId;
                    if (!m_senderTabs.contains(tabName)) {
                        return;
                    }
                }

                if (m_waitingLabel && m_waitingLabel->isVisible()) {
                    m_waitingLabel->hide();
                    if (m_classTabWidget) {
                        m_classTabWidget->show();
                    }
                }

                QString tabName = className + ":" + sId;
                if (!m_senderTabs.contains(tabName)) {
                    SenderTab* tab = new SenderTab(sId, className, m_dict, this);
                    m_classTabWidget->addTab(tab, tabName);
                    m_senderTabs[tabName] = tab;
                }

                m_senderTabs[tabName]->handleMessage(msg);
            });
        }
    }
}

/**
 * @brief Application entry point.
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return Exit status code.
 */
int main(int argc, char *argv[])
{
    // Set metadata BEFORE application instantiation to prevent XDG portal double-registration 
    // root cause ("Connection already associated with an application ID").
    QCoreApplication::setApplicationVersion("1.0");
    // Set internal names in lowercase with underscores for safe XDG folder paths
    //QCoreApplication::setOrganizationName("paparazzi"); // only for settings, not really relevant here  
    // Mint de XDG underscore for filesystem compatibility
    QGuiApplication::setDesktopFileName(QStringLiteral("paparazzi_messages"));
    QCoreApplication::setApplicationName(QStringLiteral("paparazzi-messages"));

    QApplication app(argc, argv);

    QCommandLineParser parser;
    parser.setSingleDashWordOptionMode(QCommandLineParser::ParseAsLongOptions);
    parser.setApplicationDescription("Paparazzi Messages Viewer");
    parser.addHelpOption();
    parser.addVersionOption();

    QCommandLineOption ivyBusOption("b", "Ivy bus (default 127.255.255.255:2010)", "bus", qEnvironmentVariable("IVY_BUS", "127.255.255.255:2010"));
    parser.addOption(ivyBusOption);
    QCommandLineOption classOption("c", "Class name to listen to (can be used multiple times, e.g. telemetry:*)", "class");
    parser.addOption(classOption);
    QCommandLineOption timestampOption("timestamp", "Bind to timestamped messages (currently ignored)");
    parser.addOption(timestampOption);
    QCommandLineOption forceOption("force", "Force waiting on all messages, not only ALIVE for telemetry class");
    parser.addOption(forceOption);
    QCommandLineOption geometryOption("g", "Set the window geometry (e.g., '500x500+100+100')", "geometry");
    parser.addOption(geometryOption);

    QStringList args = app.arguments();
    QStringList mergedArgs;
    for (int i = 0; i < args.size(); ++i) {
        QString arg = args[i];
        if ((arg.startsWith('\'') && !arg.endsWith('\'')) || (arg.startsWith('"') && !arg.endsWith('"'))) {
            QChar quoteType = arg[0];
            QString merged = arg;
            int j = i + 1;
            bool foundClosed = false;
            while (j < args.size()) {
                merged += " " + args[j];
                if (args[j].endsWith(quoteType)) {
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

    MessagesConfig config;
    config.ivyBus = parser.value(ivyBusOption);
    config.classes = parser.values(classOption);
    config.timestamp = parser.isSet(timestampOption);
    config.force = parser.isSet(forceOption);
    config.geometry = parser.value(geometryOption);

    QString iconPath = ":/penguin_icon_msg.png";
    QIcon icon(iconPath);
    installLinuxDesktopIntegration(app.desktopFileName(), "Paparazzi Messages", "View and inspect telemetry messages sent from or to Aircraft or elsewhere", iconPath, "paparazzi-messages");

    app.setWindowIcon(icon);
    
    MessagesWindow window(config);
    window.setWindowIcon(icon);
    window.show();

    return app.exec();
}

#include "messages.moc"

