import re
import sys

with open("src/MainWindow.cpp", "r") as f:
    content = f.read()

# Replace #include <QTabWidget> with #include <QListWidget> ...
content = content.replace('#include <QTabWidget>', '#include <QTabWidget>\n#include <QListWidget>\n#include <QStackedWidget>\n#include <QSplitter>')

# Replace SenderTab ctor
old_ctor = """SenderTab::SenderTab(const QString& senderName, const QString& className, pprzlink::MessageDictionary* dict, QWidget* parent)
    : QWidget(parent), m_senderName(senderName), m_className(className), m_dict(dict) {
    auto layout = new QVBoxLayout(this);
    m_messagesTabWidget = new QTabWidget(this);
    m_messagesTabWidget->setTabPosition(QTabWidget::West);
    layout->addWidget(m_messagesTabWidget);
}"""

new_ctor = """SenderTab::SenderTab(const QString& senderName, const QString& className, pprzlink::MessageDictionary* dict, QWidget* parent)
    : QWidget(parent), m_senderName(senderName), m_className(className), m_dict(dict) {
    auto layout = new QHBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    
    QSplitter* splitter = new QSplitter(Qt::Horizontal, this);
    
    m_listWidget = new QListWidget(this);
    m_listWidget->setSelectionMode(QAbstractItemView::SingleSelection);
    // minimal styling to look like a flat list
    m_listWidget->setStyleSheet("QListWidget { border: none; } QListWidget::item { border-bottom: 1px solid #ccc; padding: 4px; }");
    
    m_stackedWidget = new QStackedWidget(this);
    
    splitter->addWidget(m_listWidget);
    splitter->addWidget(m_stackedWidget);
    splitter->setStretchFactor(0, 1);
    splitter->setStretchFactor(1, 4);
    
    layout->addWidget(splitter);
    
    connect(m_listWidget, &QListWidget::currentRowChanged, m_stackedWidget, &QStackedWidget::setCurrentIndex);
    
    QTimer* globalTimer = new QTimer(this);
    connect(globalTimer, &QTimer::timeout, this, &SenderTab::updateTimers);
    globalTimer->start(1000);
}

void SenderTab::updateTimers() {
    for(auto& t : m_msgTrackers) {
        int secs = t.timeLabel->property("lastUpdate").toTime().secsTo(QTime::currentTime());
        if (secs < 0 || secs > 99999) secs = 0; // Just in case of midnight wrap
        t.timeLabel->setText(QString::number(secs));
    }
}
"""
content = content.replace(old_ctor, new_ctor)

# Now we need to replace handleMessage body
old_handle_start = "void SenderTab::handleMessage(const pprzlink::Message& msg) {"
new_handle_body = """void SenderTab::handleMessage(const pprzlink::Message& msg) {
    QString msgName = msg.getDefinition().getName();
    
    if (!m_msgTrackers.contains(msgName)) {
        QWidget* page = new QWidget(this);
        QVBoxLayout* vlayout = new QVBoxLayout(page);
        
        // Item in list widget
        QListWidgetItem* item = new QListWidgetItem();
        
        // Custom widget for list item
        QWidget* itemWidget = new QWidget();
        QHBoxLayout* itemLayout = new QHBoxLayout(itemWidget);
        itemLayout->setContentsMargins(4, 2, 4, 2);
        
        QLabel* nameLabel = new QLabel(msgName);
        nameLabel->setAlignment(Qt::AlignCenter);
        
        QLabel* timeLabel = new QLabel("0");
        timeLabel->setMinimumWidth(30);
        timeLabel->setAlignment(Qt::AlignCenter);
        timeLabel->setProperty("lastUpdate", QTime::currentTime());
        
        QWidget* timeBox = new QWidget();
        QHBoxLayout* tBoxL = new QHBoxLayout(timeBox);
        tBoxL->setContentsMargins(2, 2, 2, 2);
        tBoxL->addWidget(timeLabel);
        timeBox->setStyleSheet("QWidget { background-color: #00ff00; }");
        
        itemLayout->addStretch();
        itemLayout->addWidget(nameLabel);
        itemLayout->addStretch();
        itemLayout->addWidget(timeBox);
        
        item->setSizeHint(itemWidget->sizeHint());
        
        // Find insert position alphabetically
        int insertRow = 0;
        for (; insertRow < m_listWidget->count(); ++insertRow) {
            QWidget* iw = m_listWidget->indexWidget(m_listWidget->item(insertRow));
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
    tracker.timeLabel->setText("0");
    
    // Briefly flash green background
    tracker.timeBox->setStyleSheet("QWidget { background-color: #008000; }");
    QTimer::singleShot(200, tracker.timeBox, [tracker]() {
        tracker.timeBox->setStyleSheet("QWidget { background-color: #00ff00; }");
    });

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
}"""
content = "".join(content.split("void SenderTab::handleMessage(const pprzlink::Message& msg) {")[0]) + new_handle_body + "\n\nMainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {" + "".join(content.split("MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {")[1])

with open("src/MainWindow.cpp", "w") as f:
    f.write(content)
