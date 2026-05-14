#ifndef CHECKLIST_H
#define CHECKLIST_H

#include <QFrame>
#include "airframe.h"

namespace Ui {
class Checklist;
}

class Checklist : public QFrame
{
    Q_OBJECT

public:
    explicit Checklist(QString ac_id, QWidget *parent = nullptr);
    ~Checklist();

private:
    void sendMessage(QString ac_id, ChecklistItem *item);

    QString pprzlink_id;
    Ui::Checklist *ui;
};

#endif // CHECKLIST_H
