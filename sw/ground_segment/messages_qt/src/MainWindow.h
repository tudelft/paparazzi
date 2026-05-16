#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTabWidget>
#include <QMap>
#include <QString>
#include "pprzlinkQt/Message.h"

class QListWidget;
class QStackedWidget;
class QLabel;

namespace pprzlink {
    class MessageDictionary;
    class IvyQtLink;
}

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
    
    QMap<QString, MsgTracker> m_msgTrackers;
    QMap<QString, QMap<QString, class QLabel*>> m_fieldLabels;
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
    pprzlink::MessageDictionary* m_dict;
    pprzlink::IvyQtLink* m_link;

    void setupDictionaryAndLink();
};

#endif // MAINWINDOW_H