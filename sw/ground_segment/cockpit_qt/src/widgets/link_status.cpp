#include "link_status.h"
#include "AircraftManager.h"

LinkStatus::LinkStatus(QString ac_id, QWidget *parent) : QWidget(parent),
    ac_id(ac_id)
{
    grid_layout = new QGridLayout(this);
    grid_layout->setContentsMargins(5, 5, 5, 5);

    t_link_id = new QLabel("Link id:", this);
    t_status = new QLabel("Status:", this);
    t_ping_time = new QLabel("Ping time[ms]", this);
    t_link_rx = new QLabel("Link Rx [Bytes/s]", this);
    t_downlink = new QLabel("Downlink [Byte/s]", this);
    t_uplink_lost = new QLabel("Uplink lost [s]", this);

    t_link_id->setAlignment(Qt::AlignTop | Qt::AlignLeft);
    t_status->setAlignment(Qt::AlignTop | Qt::AlignLeft);
    t_ping_time->setAlignment(Qt::AlignTop | Qt::AlignLeft);
    t_link_rx->setAlignment(Qt::AlignTop | Qt::AlignLeft);
    t_downlink->setAlignment(Qt::AlignTop | Qt::AlignLeft);
    t_uplink_lost->setAlignment(Qt::AlignTop | Qt::AlignLeft);

    grid_layout->addWidget(t_link_id,     0, 0, Qt::AlignTop | Qt::AlignLeft);
    grid_layout->addWidget(t_status,      1, 0, Qt::AlignTop | Qt::AlignLeft);
    grid_layout->addWidget(t_ping_time,   2, 0, Qt::AlignTop | Qt::AlignLeft);
    grid_layout->addWidget(t_link_rx,     3, 0, Qt::AlignTop | Qt::AlignLeft);
    grid_layout->addWidget(t_downlink,    4, 0, Qt::AlignTop | Qt::AlignLeft);
    grid_layout->addWidget(t_uplink_lost, 5, 0, Qt::AlignTop | Qt::AlignLeft);
    
    grid_layout->setRowStretch(6, 1);
    grid_layout->setAlignment(Qt::AlignTop | Qt::AlignLeft);

    auto ac_status = AircraftManager::get()->getAircraft(ac_id)->getStatus();
    connect(ac_status, &AircraftStatus::telemetry_status, this, &LinkStatus::updateData);
}

void LinkStatus::updateData() {
    auto msgs = AircraftManager::get()->getAircraft(ac_id)->getStatus()->getTelemetryMessages();

    if(!msgs.contains("no_id") && links.contains("no_id")) {
        auto lni = links["no_id"];
        lni.link_id->deleteLater();
        lni.status->deleteLater();
        lni.ping_time->deleteLater();
        lni.link_rx->deleteLater();
        lni.downlink->deleteLater();
        lni.uplink_lost->deleteLater();
        links.remove("no_id");
    }


    for(auto it=msgs.begin(); it!=msgs.end(); ++it) {
        QString link_id = it.key();
        QString display_id = link_id;
        bool is_num = false;
        int id_val = link_id.toInt(&is_num);
        if (is_num && id_val == -1) {
            display_id = "single";
        }

        if(!links.contains(link_id)) {
            Status l = {
                new QLabel(display_id, this),
                new ColorLabel(0, this),
                new QLabel(this),
                new QLabel(this),
                new QLabel(this),
                new QLabel(this)
            };
            links[link_id] = l;

            l.link_id->setStyleSheet("font-weight: bold;");

            l.link_id->setAlignment(Qt::AlignTop | Qt::AlignHCenter);
            //l.status is a ColorLabel, handle alignment in layout instead
            l.ping_time->setAlignment(Qt::AlignTop | Qt::AlignHCenter);
            l.link_rx->setAlignment(Qt::AlignTop | Qt::AlignHCenter);
            l.downlink->setAlignment(Qt::AlignTop | Qt::AlignHCenter);
            l.uplink_lost->setAlignment(Qt::AlignTop | Qt::AlignHCenter);

            int col = grid_layout->columnCount();
            grid_layout->addWidget(l.link_id, 0, col, Qt::AlignTop | Qt::AlignHCenter);
            grid_layout->addWidget(l.status, 1, col, Qt::AlignTop | Qt::AlignHCenter);
            grid_layout->addWidget(l.ping_time, 2, col, Qt::AlignTop | Qt::AlignHCenter);
            grid_layout->addWidget(l.link_rx, 3, col, Qt::AlignTop | Qt::AlignHCenter);
            grid_layout->addWidget(l.downlink, 4, col, Qt::AlignTop | Qt::AlignHCenter);
            grid_layout->addWidget(l.uplink_lost, 5, col, Qt::AlignTop | Qt::AlignHCenter);
        }

        auto l = links[link_id];

        float time_since_last_msg = 0, ping_time = 0, rx_bytes_rate = 0;
        uint16_t downlink_rate = 0;
        uint32_t uplink_lost_time = 0;
        try { it.value().getField("time_since_last_msg", time_since_last_msg); } catch(...) {}
        try { it.value().getField("ping_time", ping_time); } catch(...) {}
        try { it.value().getField("rx_bytes_rate", rx_bytes_rate); } catch(...) {}
        try { it.value().getField("downlink_rate", downlink_rate); } catch(...) {}
        try { it.value().getField("uplink_lost_time", uplink_lost_time); } catch(...) {}

        QString status_txt = "";
        if(time_since_last_msg > 2) {
            status_txt = QString::number(static_cast<int>(time_since_last_msg));
        }
        l.status->setText(status_txt);
        if(time_since_last_msg < 5) {
            l.status->setBrush(Qt::green);
        } else {
            l.status->setBrush(Qt::red);
        }

        l.link_id->setText(display_id);
        l.link_rx->setText(QString::number(rx_bytes_rate));
        l.ping_time->setText(QString::number(ping_time));
        l.downlink->setText(QString::number(downlink_rate));
        l.uplink_lost->setText(QString::number(uplink_lost_time));
    }
}
