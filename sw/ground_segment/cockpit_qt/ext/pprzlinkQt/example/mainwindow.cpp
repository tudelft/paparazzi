#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include "pprzlinkQt/MessageDictionary.h"
#include "pprzlinkQt/Message.h"
#include "pprzlinkQt/IvyQtLink.h"
#include "iostream"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    auto dict = new pprzlink::MessageDictionary("/home/fabien/paparazzi/var/messages.xml");

    auto link = new pprzlink::IvyQtLink(*dict, "test", this);

    link->start("127.255.255.255:2010");

    auto def_fp = dict->getDefinition("FLIGHT_PARAM");

    connect(link, &pprzlink::IvyQtLink::serverConnected, this, [=]() {
       qDebug() << "server connected!!!" ;
    });


    connect(ui->start_btn, &QPushButton::clicked, this, [=]() {
        link->start();
    });

    connect(ui->stop_btn, &QPushButton::clicked, this, [=]() {
        link->stop();
    });


    link->BindMessage(def_fp, ui->label, [=](QString, pprzlink::Message msg){
        float alt, lat, lon;
        msg.getField("ac_id", last_ac_id);
        msg.getField("alt", alt);
        msg.getField("lat", lat);
        msg.getField("long", lon);
        qDebug() << last_ac_id + " " + QString::number(alt) + "m" + "  " + QString::number(lat) + " " + QString::number(lon);
        ui->label->setText(last_ac_id + " " + QString::number(alt) + "m" + "  " + QString::number(lat) + " " + QString::number(lon));
    });

    connect(ui->label_button, &QPushButton::clicked, ui->label, [=]() {
        ui->label->deleteLater();
    });


    connect(ui->line_edit, &QLineEdit::returnPressed, this, [=]() {
        bool ok = false;
        uint8_t block_id = ui->line_edit->text().toUInt(&ok);
        if(ok) {
            auto def = dict->getDefinition("JUMP_TO_BLOCK");
            auto msg = pprzlink::Message(def);
            msg.addField("ac_id", last_ac_id);
            msg.addField("block_id", block_id);

            msg.setSenderId("ground");

            link->sendMessage(msg);
        } else {
            qDebug() << "not a number!!!";
        }
    });
}

MainWindow::~MainWindow()
{
    delete ui;
}
