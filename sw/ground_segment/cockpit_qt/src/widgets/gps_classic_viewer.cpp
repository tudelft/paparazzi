#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QScrollArea>
#include <QDebug>
#include "gps_classic_viewer.h"
#include "AircraftManager.h"
#include "setting_menu.h"
#include "setting.h"

GPSClassicViewer::GPSClassicViewer(QString ac_id, QWidget *parent) : QWidget(parent),
    ac_id(ac_id), reduced(true)
{
    connect(AircraftManager::get()->getAircraft(ac_id)->getStatus(),
            &AircraftStatus::engine_status, this, [=]() {
        this->update();
    });
}

void GPSClassicViewer::mousePressEvent(QMouseEvent *event) {
    (void)event;
    reduced = !reduced;
}

void GPSClassicViewer::paintEvent(QPaintEvent *event) {
    (void)event;
    QPainter p(this);

    auto [pacc, infos] = getData();

    int nb_sat_used = 0;
    for(auto &info: infos) {
        if(info.flags & 0x01) {
            nb_sat_used++;
        }
    }


    // Original classic behavior: Solid white background
    p.setBrush(Qt::white);

    //Gray background if no accuracy, reddish if bad, orange if minimal, green if good
    // if(pacc > 800 || nb_sat_used <= 4) {
    //     p.setBrush(QColor(0xff8888));
    // } else if(pacc > 400 || nb_sat_used <= 6) {
    //     p.setBrush(QColor(0xffc088));
    // } else {
    //     p.setBrush(QColor(0x88ff88));  /Should be theme ? dependant
    // }

    p.setPen(Qt::NoPen);
    p.drawRect(rect());

    int ypacc = 0;

    if(!reduced) {
        minSize = QSize(infos.size() * SV_WIDTH, 20 + 90 + 20 + 20 + 20);
        setMinimumSize(minSize);

        const int yId = 20 + 90;
        ypacc = yId + 20;

        for(int i=0; i<infos.size(); i++) {
            int id = infos[i].id;
            int flags = infos[i].flags;
            int cno = infos[i].cno;
            int age = infos[i].age;

            p.setPen(Qt::black);

            auto rect_cno = QRect(SV_WIDTH*i, 0, SV_WIDTH, 20);
            p.drawText(rect_cno, Qt::AlignCenter, QString::number(cno), &rect_cno);


            p.setPen(Qt::NoPen);
            if(age > 5) {
                p.setBrush(QColor(0x888888));
            } else {
                if(flags & 0x01) {
                    p.setBrush(QColor(0x00a000));
                } else {
                    p.setBrush(QColor(0xcc0000));
                }
            }

            int s = (qMax(20, cno) - 20) * 3;

            int bar_left = SV_WIDTH * i + 5;
            int bar_width = SV_WIDTH - 10;
            int bar_top = yId - 5 - s;

            if (s > 0) {
                p.drawRect(bar_left, bar_top, bar_width, s);
            }

            p.setPen(Qt::black);
            auto rect_id = QRect(SV_WIDTH*i, yId, SV_WIDTH, 20);
            p.drawText(rect_id, Qt::AlignCenter, QString::number(id), &rect_id);
        }
    } else {
        minSize = QSize(200, 40);
        setMinimumSize(minSize);
    }


    auto rect_acc = QRect(0, ypacc, rect().width(), 20);
    
    // Pacc bar background
    if (pacc != 65535 && pacc != 0) {
        int max_pacc = 2000;
        int bar_w = qMin(rect().width(), (pacc * rect().width()) / max_pacc);
        p.setBrush(QColor(Qt::red));
        p.setPen(Qt::NoPen);
        p.drawRect(0, ypacc, bar_w, 20);
    }

    QString txt;
    if (pacc == 65535 || pacc == 0) {
        txt = "Pos accuracy: N/A";
    } else {
        txt = QString("Pos accuracy: %1m").arg(pacc/100., 0, 'f', 1);
    }

    QFont f;
    f.setBold(true);
    p.setFont(f);
    p.setPen(Qt::black);
    // Put the text roughly centered, or bounded by width
    p.drawText(rect_acc, Qt::AlignCenter, txt, &rect_acc);

    auto rect_nb_used = QRect(0, ypacc + 20, rect().width(), 20);
    p.drawText(rect_nb_used, Qt::AlignCenter, QString("%1 satellites used").arg(nb_sat_used), &rect_nb_used);

}

QSize GPSClassicViewer::sizeHint() const
{
    return minimumSizeHint();
}

QSize GPSClassicViewer::minimumSizeHint() const
{
    return QSize(qMax(minSize.width(), 200), qMax(minSize.height(), 40));
}


std::tuple<uint16_t, QList<struct GPSClassicViewer::SvInfo>> GPSClassicViewer::getData() {
    auto msg = AircraftManager::get()->getAircraft(ac_id)->getStatus()->getMessage("SVSINFO");
    if(!msg) {
        return make_tuple(65535, QList<struct SvInfo>());
    }

    uint16_t pacc;
    QString _svids, _flags, _qis, _cnos, _elevs, _azims, _msg_ages;

    msg->getField("pacc", pacc);
    msg->getField("svid", _svids);
    msg->getField("flags", _flags);
    //msg->getField("qi", _qis);
    msg->getField("cno", _cnos);
    //msg->getField("elev", _elevs);
    //msg->getField("azim", _azims);
    msg->getField("msg_age", _msg_ages);

    QList<int> svids;
    QList<int> flags;
    //QList<int> qis;
    QList<int> cno;
    //QList<int> elevs;
    //QList<int> azims;
    QList<int> msg_age;

    QList< struct SvInfo> infos;

    for(auto &id: _svids.split(',')) {
        svids.append(id.toInt());
    }

//    for(auto sv: QString(_svids.c_str()).split(',')) {
//        svids.append(sv.toInt());
//    }

    for(auto &fls: _flags.split(',')) {
        flags.append(fls.toInt());
    }

    // qis
    for(auto &cn: _cnos.split(',')) {
        cno.append(cn.toInt());
    }

    // elevs

    // azims

    for(auto &age: _msg_ages.split(',')) {
        msg_age.append(age.toInt());
    }

    for(int i=0; i<svids.size(); i++) {
        infos.append(SvInfo{svids[i], flags[i], cno[i], msg_age[i]});
    }

    QMutableListIterator<struct SvInfo> i(infos);
    while (i.hasNext()) {
        auto next = i.next();
        if (next.age > 60 || next.id == 0) {
            i.remove();
        }
    }

    return make_tuple(pacc, infos);
}

GPSViewerTab::GPSViewerTab(QString ac_id, QWidget *parent) : QWidget(parent), ac_id(ac_id), gps_reset_setting(nullptr)
{
    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->setContentsMargins(5, 5, 5, 5);
    
    QScrollArea* scrollArea = new QScrollArea(this);
    scrollArea->setWidgetResizable(true);
    scrollArea->setFrameShape(QFrame::NoFrame);
    
    viewer = new GPSClassicViewer(ac_id, scrollArea);
    // Explicitly trigger reduced to false so it shows full graphical view in tab
    QMouseEvent fakeEvent(QEvent::MouseButtonPress, QPointF(0,0), QPointF(0,0), Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
    viewer->mousePressEvent(&fakeEvent); 
    
    scrollArea->setWidget(viewer);
    layout->addWidget(scrollArea);
    
    QHBoxLayout* btnLayout = new QHBoxLayout();
    QLabel* lblReset = new QLabel("Reset:", this);
    QPushButton* btnHot = new QPushButton("Hostart", this);
    QPushButton* btnWarm = new QPushButton("Warmstart", this);
    QPushButton* btnCold = new QPushButton("Coldstart", this);
    
    btnLayout->addWidget(lblReset);
    btnLayout->addWidget(btnHot);
    btnLayout->addWidget(btnWarm);
    btnLayout->addWidget(btnCold);
    btnLayout->addStretch();
    
    layout->addLayout(btnLayout);
    
    // Find gps reset setting
    auto ac = AircraftManager::get()->getAircraft(ac_id);
    if(ac && ac->getSettingMenu()) {
        for(auto s : ac->getSettingMenu()->getAllSettings()) {
            if(s->getFullName().endsWith("gps.reset") || s->getFullName().endsWith("gps_ubx_reset")) {
                gps_reset_setting = s;
                break;
            }
        }
    }
    
    if (!gps_reset_setting) {
        lblReset->hide();
        btnHot->hide();
        btnWarm->hide();
        btnCold->hide();
    } else {
        connect(btnHot, &QPushButton::clicked, this, [=]() { if(gps_reset_setting) AircraftManager::get()->getAircraft(ac_id)->setSetting(gps_reset_setting, 0.0f); });
        connect(btnWarm, &QPushButton::clicked, this, [=]() { if(gps_reset_setting) AircraftManager::get()->getAircraft(ac_id)->setSetting(gps_reset_setting, 1.0f); });
        connect(btnCold, &QPushButton::clicked, this, [=]() { if(gps_reset_setting) AircraftManager::get()->getAircraft(ac_id)->setSetting(gps_reset_setting, 2.0f); });
    }
}
