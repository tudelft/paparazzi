#ifndef CHAT_H
#define CHAT_H

#include <QFrame>

namespace Ui {
class Chat;
}

class Chat : public QFrame
{
    Q_OBJECT

public:
    explicit Chat(QWidget *parent = nullptr);
    ~Chat();

private:

    void onSend();
    //void onNewMessage();
    void addMessage(QString txt, QString source, QString dst=QString(), bool sent=false);

    void onNewAircraft(QString ac_id);
    void onAcDeleted(QString ac_id);

    QString chat_id;

    Ui::Chat *ui;
};

#endif // CHAT_H
