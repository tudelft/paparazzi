#include "messages.h"
#include <QMainWindow>
#include <QTabWidget>
#include <QMap>
#include <QString>
#include <QApplication>
#include <QListWidget>
#include <QStackedWidget>
#include <QSplitter>
#include <QListWidgetItem>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QScrollArea>
#include <QDrag>
#include <QMimeData>
#include <QMouseEvent>
#include <QDomDocument>
#include <QFile>
#include <QDebug>
#include <QTimer>
#include <QTime>
#include <cstdlib>
#include <sstream>
#include "pprzlinkQt/IvyQtLink.h"
#include "pprzlinkQt/Message.h"
#include "pprzlinkQt/MessageDictionary.h"
#include "pprzlinkQt/MessageDefinition.h"
#include "pprzlinkQt/MessageField.h"
#include "pprzlinkQt/FieldValue.h"


static QMap<QString, QMap<QString, QMap<QString, QString>>> s_unitCoefs;
static QMap<QString, QMap<QString, QMap<QString, QString>>> s_unitNames;

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
    m_listWidget->setFrameShape(QFrame::NoFrame);
    m_listWidget->setAttribute(Qt::WA_MacShowFocusRect, false);
    m_listWidget->setSelectionMode(QAbstractItemView::SingleSelection);
    // minimal styling to look like a flat list
    
    m_stackedWidget = new QStackedWidget(this);
    
    splitter->addWidget(m_listWidget);
    splitter->addWidget(m_stackedWidget);
    splitter->setStretchFactor(0, 1);
    splitter->setStretchFactor(1, 4);
    
    layout->addWidget(splitter);
    
    connect(m_listWidget, &QListWidget::currentRowChanged, m_stackedWidget, &QStackedWidget::setCurrentIndex);
    
#define GREEN_DECAY_RATE_MS 500

    QTimer* globalTimer = new QTimer(this);
    connect(globalTimer, &QTimer::timeout, this, &SenderTab::updateTimers);
    globalTimer->start(50);
}

void SenderTab::updateTimers() {
    for(auto& t : m_msgTrackers) {
        int msecs = t.timeLabel->property("lastUpdate").toTime().msecsTo(QTime::currentTime());
        if (msecs < 0 || msecs > 99999999) msecs = 0; // Just in case of midnight wrap
        
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


void SenderTab::handleMessage(const pprzlink::Message& msg) {
    QString msgName = msg.getDefinition().getName();
    
    if (!m_msgTrackers.contains(msgName)) {
        QWidget* page = new QWidget(this);
        QVBoxLayout* vlayout = new QVBoxLayout(page);
        
        // Item in list widget
        QListWidgetItem* item = new QListWidgetItem();
        
        // Custom widget for list item
        QWidget* itemWidget = new QWidget();
        itemWidget->setStyleSheet("background: transparent;");
        itemWidget->setStyleSheet("background: transparent;");
        QHBoxLayout* itemLayout = new QHBoxLayout(itemWidget);
        itemLayout->setContentsMargins(4, 2, 4, 2);
        
        QLabel* nameLabel = new QLabel(msgName);
        nameLabel->setAlignment(Qt::AlignCenter);
        
        QLabel* timeLabel = new QLabel("");
        timeLabel->setMinimumWidth(30);
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
        
        item->setSizeHint(itemWidget->sizeHint());
        
        // Find insert position alphabetically
        int insertRow = 0;
        for (; insertRow < m_listWidget->count(); ++insertRow) {
            QWidget* iw = m_listWidget->itemWidget(m_listWidget->item(insertRow));
            if (iw) {
                QLabel* nL = iw->findChild<QLabel*>();
                if (nL && nL->text() > msgName) break;
            }
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
            
            QString fieldName = field.getName();
            QString typeName = field.getType().toString();
            
            QString coef = s_unitCoefs[m_className][msgName][fieldName];
            if (coef.isEmpty()) coef = "1.";
            QString unit = s_unitNames[m_className][msgName][fieldName];
            
            QString btnText = typeName + " " + fieldName + (unit.isEmpty() ? "" : ": (" + unit + ")");
            QString payload = m_senderName + ":" + m_className + ":" + msgName + ":" + fieldName + ":" + coef;
            
            DraggableButton* btn = new DraggableButton(btnText, payload);
            QLabel* valLabel = new QLabel("XXXX");
            
            hlayout->addWidget(btn);
            hlayout->addWidget(valLabel);
            hlayout->addStretch();
            
            vlayout->addLayout(hlayout);
            m_fieldLabels[msgName][fieldName] = valLabel;
        }
        vlayout->addStretch();
    }
    
    // Update values
    MsgTracker& tracker = m_msgTrackers[msgName];
    tracker.timeLabel->setProperty("lastUpdate", QTime::currentTime());
    tracker.timeLabel->setText("");
    
    // Briefly flash green background
    tracker.timeBox->setStyleSheet(".QWidget { background-color: #22ff22; border-radius: 0px; }\nQLabel { color: #000; font-weight: bold; }");
    
    const auto& def = msg.getDefinition();
    for (int i = 0; i < (int)def.getNbFields(); ++i) {
        QString name = def.getField(i).getName();
        try {
            const auto& rv = msg.getRawValue(i);
            std::stringstream ss;
            ss << rv;
            QString s = QString::fromStdString(ss.str());
            
            QLabel* lbl = m_fieldLabels[msgName][name];
            if (lbl) lbl->setText(s);
        } catch(...) {
        }
    }
}

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
    setWindowTitle("Telemetry Messages");
    resize(300, 400);

    QWidget* cntral = new QWidget(this);
    QVBoxLayout* layout = new QVBoxLayout(cntral);
    layout->setContentsMargins(0, 0, 0, 0);

    m_waitingLabel = new QLabel("Waiting for telemetry", this);
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
    
    loadUnitCoefs(xmlPath);
    
    m_dict = new pprzlink::MessageDictionary(xmlPath);
    m_link = new pprzlink::IvyQtLink(*m_dict, "messages_qt", this);
    m_link->start("127.255.255.255:2010");
    
    QString className = "telemetry";
    
    const auto msgs = m_dict->getMsgsForClass(className);
    for (const auto& def : msgs) {
        m_link->BindMessage(def, this, [=](QString sender, pprzlink::Message msg) {
            if (m_waitingLabel->isVisible()) {
                m_waitingLabel->hide();
                m_classTabWidget->show();
            }
            
            QString sId = sender;
            if (sId.isEmpty()) {
                // telemetry usually starts with AC_ID like "2", get it dynamically
                const auto& senderV = msg.getSenderId();
                if (std::holds_alternative<QString>(senderV)) sId = std::get<QString>(senderV);
                else sId = QString::number(std::get<uint8_t>(senderV));
            }
            if (sId.isEmpty()) sId = "ground";
            
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

int main(int argc, char *argv[])
{
    // Force GTK3 platform theme which natively supports Ubuntu Adwaita dark/light
    //qputenv("QT_QPA_PLATFORMTHEME", "gtk3");
    
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}

