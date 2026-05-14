#ifndef CHATBUBBLE_H
#define CHATBUBBLE_H

#include <QWidget>
#include <QTime>

namespace Ui {
class ChatBubble;
}

class ChatBubble : public QWidget
{
    Q_OBJECT

public:
    explicit ChatBubble(QString source, QString dest, QString msg, bool sent=false, QWidget *parent = nullptr);
    ~ChatBubble();

    QString getSource() {return source;}
    QString getDest() {return dest;}
    QTime getTime() {return chat_time;}
    void addText(QString text);

protected:
    bool eventFilter(QObject *obj, QEvent *ev) override;

private:
    Ui::ChatBubble *ui;

    QString source;
    QString dest;
    QTime chat_time;
};

#endif // CHATBUBBLE_H
