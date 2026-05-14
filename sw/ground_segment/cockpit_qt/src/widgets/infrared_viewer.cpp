#include <QVBoxLayout>
#include <QHBoxLayout>
#include "infrared_viewer.h"
#include "AircraftManager.h"

InfraredViewer::InfraredViewer(QString ac_id, QWidget *parent) : QWidget(parent), ac_id(ac_id)
{
    auto main_layout = new QVBoxLayout(this);
    main_layout->setContentsMargins(5, 5, 5, 5);
    
    // IR contrast layout
    auto hbox = new QHBoxLayout();
    auto ir_contrast = new QVBoxLayout();
    ir_contrast->addWidget(new QLabel("IR\ncontrast"));
    
    ir_left = new QProgressBar(this); ir_left->setOrientation(Qt::Vertical);
    ir_front = new QProgressBar(this); ir_front->setOrientation(Qt::Vertical);
    ir_top = new QProgressBar(this); ir_top->setOrientation(Qt::Vertical);
    
    ir_contrast->addWidget(new QLabel("left")); ir_contrast->addWidget(ir_left);
    ir_contrast->addWidget(new QLabel("front")); ir_contrast->addWidget(ir_front);
    ir_contrast->addWidget(new QLabel("top")); ir_contrast->addWidget(ir_top);
    
    hbox->addLayout(ir_contrast);

    // Roll and Pitch
    auto att_lay = new QVBoxLayout();
    roll_bar = new QProgressBar(this);
    pitch_bar = new QProgressBar(this);
    att_lay->addWidget(new QLabel("Roll")); att_lay->addWidget(roll_bar);
    att_lay->addWidget(new QLabel("Pitch")); att_lay->addWidget(pitch_bar);
    hbox->addLayout(att_lay);

    main_layout->addLayout(hbox);

    // Table for other variables
    auto table = new QGridLayout();
    table->addWidget(new QLabel("contrast status"), 0, 0);
    contrast_status = new QLabel("");
    table->addWidget(contrast_status, 0, 1);

    table->addWidget(new QLabel("contrast"), 1, 0);
    contrast_value = new QLabel("");
    table->addWidget(contrast_value, 1, 1);

    table->addWidget(new QLabel("gps hybrid mode"), 2, 0);
    gps_hybrid_mode = new QLabel("");
    table->addWidget(gps_hybrid_mode, 2, 1);

    table->addWidget(new QLabel("gps hybrid factor"), 3, 0);
    gps_hybrid_factor = new QLabel("");
    table->addWidget(gps_hybrid_factor, 3, 1);

    main_layout->addLayout(table);

    // Connect to status if available
    auto ac_status = AircraftManager::get()->getAircraft(ac_id)->getStatus();
    connect(ac_status, &AircraftStatus::telemetry_status, this, &InfraredViewer::updateData);
}

void InfraredViewer::setContrastStatus(const QString& s) { contrast_status->setText(s); }
void InfraredViewer::setContrastValue(int s) { contrast_value->setText(QString::number(s)); }
void InfraredViewer::setGpsHybridMode(const QString& s) { gps_hybrid_mode->setText(s); }
void InfraredViewer::setGpsHybridFactor(float s) { gps_hybrid_factor->setText(QString::number(s, 'f', 8)); }

void InfraredViewer::updateData() {
    auto msgs = AircraftManager::get()->getAircraft(ac_id)->getStatus()->getTelemetryMessages();
    if(msgs.contains("INFRARED")) {
        auto msg = msgs["INFRARED"];
        int16_t ir1, ir2, ir3;
        try { msg.getField("ir1", ir1); ir_left->setValue(ir1); } catch(...) {}
        try { msg.getField("ir2", ir2); ir_front->setValue(ir2); } catch(...) {}
        try { msg.getField("ir3", ir3); ir_top->setValue(ir3); } catch(...) {}
    }
}
