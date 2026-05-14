#ifndef LINKSTATUS_H
#define LINKSTATUS_H

#include <QtWidgets>
#include "colorlabel.h"

class LinkStatus : public QWidget
{
    Q_OBJECT
public:
    explicit LinkStatus(QString ac_id, QWidget *parent = nullptr);

signals:

private:

    void updateData();

    struct Status {
        QLabel* link_id;
        ColorLabel* status;
        QLabel* ping_time;
        QLabel* link_rx;
        QLabel* downlink;
        QLabel* uplink_lost;
    };

    QString ac_id;

    QGridLayout* grid_layout;

    QLabel* t_link_id;
    QLabel* t_status;
    QLabel* t_ping_time;
    QLabel* t_link_rx;
    QLabel* t_downlink;
    QLabel* t_uplink_lost;

    QMap<QString, Status> links;

};

#endif // LINKSTATUS_H
