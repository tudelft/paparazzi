#include "ac_selector.h"
#include "dispatcher_ui.h"
#include <QPainter>
#include "dispatcher_ui.h"
#include "AircraftManager.h"
#include <QDebug>

ACSelector::ACSelector(QWidget *parent) : QWidget(parent)
{

    connect(DispatcherUi::get(), &DispatcherUi::new_ac_config, this, &ACSelector::handleNewAC);
    connect(DispatcherUi::get(), &DispatcherUi::ac_deleted, this, &ACSelector::removeAC);
    connect(DispatcherUi::get(), &DispatcherUi::ac_selected, this,
            [=](QString ac_id) {
                current_ac_id = ac_id;
                repaint();
            });
}

void ACSelector::handleNewAC(QString ac_id) {
    ac_ids.append(ac_id);
    repaint();
}

void ACSelector::removeAC(QString ac_id) {
    ac_ids.removeAll(ac_id);
    repaint();
}

void ACSelector::paintEvent(QPaintEvent* e) {
    (void)e;
    QPainter painter(this);
    painter.setPen(Qt::NoPen);

    QRect arrow_left_rect = QRect(0, rect().height()/2, ARROW_WIDTH, rect().height()/2);
    painter.setBrush(Qt::white);
    painter.drawRect(arrow_left_rect);

    QRect arrow_right_rect = QRect(rect().width()-ARROW_WIDTH, rect().height()/2, ARROW_WIDTH, rect().height()/2);
    painter.setBrush(Qt::white);
    painter.drawRect(arrow_right_rect);

    painter.setPen(Qt::black);
    painter.drawText(arrow_left_rect, Qt::AlignCenter, "<");
    painter.drawText(arrow_right_rect, Qt::AlignCenter, ">");

    if(ac_ids.size() == 0) {
        painter.setPen(Qt::NoPen);
        painter.setBrush(QColor(0x808080));
        auto r = QRect(ARROW_WIDTH, rect().height()/2, rect().width()-2*ARROW_WIDTH, rect().height()/2);
        painter.drawRect(r);
        painter.setPen(Qt::white);
        painter.drawText(r.marginsRemoved(QMargins(5, 0, 5, 0)), Qt::AlignCenter, QString::fromUtf8("\xE2\x88\x85"), &r);
    }

    for(int i=0; i<ac_ids.size(); i++) {
        auto ac_id = ac_ids[i];
        auto left = arrow_left_rect.width() + (i * (rect().width()-2*ARROW_WIDTH)) / ac_ids.size();
        auto right = arrow_left_rect.width() + ((i+1) * (rect().width()-2*ARROW_WIDTH)) / ac_ids.size();
        int width = right - left;
        auto top = ac_id == current_ac_id ? 0: rect().height()/2;
        auto height = ac_id == current_ac_id ? rect().height(): rect().height()/2;
        QRect rect = QRect(left, top, width, height);
        auto ac = AircraftManager::get()->getAircraft(ac_id);
        painter.setPen(Qt::NoPen);
        painter.setBrush(ac->getColor());
        painter.drawRoundedRect(rect, 10, 10);

        auto dark = QColor(0x101010);
        auto light = QColor(0xf0f0f0);

        auto L1 = 0.2126 * (ac->getColor().red()/255.) + 0.7152 * (ac->getColor().green()/255.) + 0.0722 * (ac->getColor().blue()/255.);
        auto Ldark = 0.2126 * (dark.red()/255.) + 0.7152 * (dark.green()/255.) + 0.0722 * (dark.blue()/255.);
        auto Llight = 0.2126 * (light.red()/255.) + 0.7152 * (light.green()/255.) + 0.0722 * (light.blue()/255.);

        auto crDark = (L1 + 0.05) / (Ldark + 0.05);
        auto crLight = (Llight + 0.05) / (L1 + 0.05);

        auto txtColor = crDark > crLight ? dark : light;

        painter.setPen(txtColor);
        painter.drawText(rect.marginsRemoved(QMargins(5, 0, 5, 0)), Qt::AlignVCenter, ac->name(), &rect);

    }
}

void ACSelector::mousePressEvent(QMouseEvent *e) {
    if(ac_ids.size() == 0) {
        return;
    }

    QRect arrow_left_rect = QRect(0, this->rect().height()/2, ARROW_WIDTH, this->rect().height()/2);
    QRect arrow_right_rect = QRect(this->rect().width()-ARROW_WIDTH, this->rect().height()/2, ARROW_WIDTH, this->rect().height()/2);

    if(arrow_left_rect.contains(e->pos())) {
        auto index = ac_ids.indexOf(current_ac_id) - 1;
        if(index >= 0) {
            emit DispatcherUi::get()->ac_selected(ac_ids[index]);
        }
    } else if (arrow_right_rect.contains(e->pos())) {
        auto index = ac_ids.indexOf(current_ac_id) + 1;
        if(index < ac_ids.size()) {
            emit DispatcherUi::get()->ac_selected(ac_ids[index]);
        }
    } else {
        QString pressed_ac_id = "";

        auto width = (this->rect().width() - 2*ARROW_WIDTH) / ac_ids.size();
        for(int i=0; i<ac_ids.size(); i++) {
            auto ac_id = ac_ids[i];
            auto left = ARROW_WIDTH + i * width;
            auto top = ac_id == current_ac_id ? 0: this->rect().height()/2;
            auto height = ac_id == current_ac_id ? this->rect().height(): this->rect().height()/2;
            QRect rect = QRect(left, top, width, height);

            if(rect.contains(e->pos())) {
                pressed_ac_id = ac_id;
                break;
            }
        }

        if(pressed_ac_id != "") {
            emit DispatcherUi::get()->ac_selected(pressed_ac_id);
        }
    }
}

void ACSelector::wheelEvent(QWheelEvent *event) {
    auto incr = event->angleDelta().y() > 0 ? 1 : -1;
    auto index = ac_ids.indexOf(current_ac_id) + incr;
    if(index >= 0 && index < ac_ids.size()) {
        emit DispatcherUi::get()->ac_selected(ac_ids[index]);
    }
}

void ACSelector::mouseReleaseEvent(QMouseEvent *e) {
    (void)e;
}

QSize ACSelector::sizeHint() const
{
    return minimumSizeHint();
}

QSize ACSelector::minimumSizeHint() const
{
    return QSize(2*ARROW_WIDTH + 40, 2*ARROW_WIDTH);
}
