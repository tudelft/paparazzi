#include "bindinspector.h"
#include "ui_bindinspector.h"

BindInspector::BindInspector(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::BindInspector)
{
    ui->setupUi(this);
}

BindInspector::~BindInspector()
{
    delete ui;
}
