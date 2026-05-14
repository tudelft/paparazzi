#include "misc_viewer.h"
#include <QVBoxLayout>

MiscViewer::MiscViewer(QString ac_id, QWidget *parent) : QWidget(parent), ac_id(ac_id)
{
    table = new QGridLayout(this);
    table->setContentsMargins(5, 5, 5, 5);
    table->setHorizontalSpacing(40);
    table->setVerticalSpacing(5);

    QStringList misc_fields = {"Wind speed", "Wind direction", "Mean airspeed", "Time to HOME", "Send periodically"};
    int row = 0;
    for (const QString& field : misc_fields) {
        QLabel *lbl = new QLabel(field, this);
        lbl->setAlignment(Qt::AlignTop | Qt::AlignHCenter);
        table->addWidget(lbl, row, 0, Qt::AlignTop | Qt::AlignHCenter);
        
        if (field == "Send periodically") {
            periodic_send = new QCheckBox(this);
            periodic_send->setChecked(true);
            table->addWidget(periodic_send, row, 1, Qt::AlignTop | Qt::AlignHCenter);
        } else {
            auto val_label = new QLabel("N/A", this);
            val_label->setAlignment(Qt::AlignTop | Qt::AlignHCenter);
            values.insert(field, val_label);
            table->addWidget(val_label, row, 1, Qt::AlignTop | Qt::AlignHCenter);
        }
        row++;
    }
    
    table->setRowStretch(row, 1);
    table->setAlignment(Qt::AlignTop | Qt::AlignLeft);
}

void MiscViewer::setValue(const QString& label, const QString& text) {
    if (values.contains(label)) {
        values[label]->setText(text);
    }
}
