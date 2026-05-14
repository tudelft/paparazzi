#include "flightplan_viewerv2.h"
#include <QVBoxLayout>
#include <QtWidgets>
#include "AircraftManager.h"
#include <QDebug>
#include <QSettings>
#include "gcs_utils.h"
#include "flightplaneditor.h"

FlightPlanViewerV2::FlightPlanViewerV2(QString ac_id, QWidget *parent) : QTabWidget(parent),
    ac_id(ac_id), current_block(0), current_stage(0), list_widget(nullptr)
{
    addTab(make_blocks_tab(), "Blocks");
    addTab(new FlightPlanEditor(ac_id, this), "Details");

    connect(AircraftManager::get()->getAircraft(ac_id)->getStatus(),
            &AircraftStatus::nav_status, this, &FlightPlanViewerV2::handleNavStatus);
}


QWidget* FlightPlanViewerV2::make_blocks_tab() {
    list_widget = new QListWidget(this);
    list_widget->setSelectionMode(QAbstractItemView::SingleSelection);
    list_widget->setFrameShape(QFrame::NoFrame);
    list_widget->setMinimumHeight(40); // Allow info tab to shrink vertically

    QFont list_font = list_widget->font();
    list_font.setPointSize(list_font.pointSize() + 2);
    list_widget->setFont(list_font);
    list_widget->setSpacing(4);

    for(auto block: AircraftManager::get()->getAircraft(ac_id)->getFlightPlan()->getBlocks()) {
        QString icon = block->getIcon();
        QString txt = block->getText();
        QString name = block->getName();

        QString display_text = txt != "" ? txt : name;
        auto item = new QListWidgetItem(display_text, list_widget);

        if (icon != "") {
            QString icon_path = appConfig()->value("GCS_ICONS_PATH").toString() + "/" + icon;
            item->setIcon(QIcon(icon_path));
            item->setToolTip(txt);
        }
        item->setData(Qt::UserRole, block->getNo());
    }

    connect(list_widget, &QListWidget::itemActivated, this, [this](QListWidgetItem* item) {
        if(item) {
            uint8_t block_no = item->data(Qt::UserRole).toUInt();
            pprzlink::Message msg(PprzDispatcher::get()->getDict()->getDefinition("JUMP_TO_BLOCK"));
            msg.addField("ac_id", ac_id);
            msg.addField("block_id", block_no);
            PprzDispatcher::get()->sendMessage(msg);
        }
    });

    return list_widget;
}


void FlightPlanViewerV2::handleNavStatus() {
    auto msg = AircraftManager::get()->getAircraft(ac_id)->getStatus()->getMessage("NAV_STATUS");
    if(msg) {
        uint8_t cur_block, cur_stage;
        //uint32_t block_time, stage_time;
        //float target_lat, target_long, target_climb, target_alt, target_course, dist_to_wp;
        msg->getField("cur_block", cur_block);
        msg->getField("cur_stage", cur_stage);

        QTimer* timer = new QTimer();
        timer->moveToThread(qApp->thread());
        timer->setSingleShot(true);
        QObject::connect(timer, &QTimer::timeout, this, [=]()
        {
            // main thread
            updateNavStatus(cur_block, cur_stage);
            timer->deleteLater();
        });
        QMetaObject::invokeMethod(timer, "start", Qt::QueuedConnection, Q_ARG(int, 0));

    }
}

void FlightPlanViewerV2::updateNavStatus(uint8_t cur_block, uint8_t cur_stage) {
    if(cur_block != current_block || cur_stage != current_stage) {
        //reset idle state
        if (current_block < list_widget->count()) {
            auto old_item = list_widget->item(current_block);
            if (old_item) {
                old_item->setBackground(Qt::NoBrush);
                old_item->setForeground(Qt::NoBrush);
            }
        }

        //set "current" state
        if (cur_block < list_widget->count()) {
            auto new_item = list_widget->item(cur_block);
            if (new_item) {
                new_item->setBackground(QBrush(Qt::darkGreen));
                new_item->setForeground(QBrush(Qt::white));
            }
        }
        current_block = cur_block;
        current_stage = cur_stage;
    }
}
