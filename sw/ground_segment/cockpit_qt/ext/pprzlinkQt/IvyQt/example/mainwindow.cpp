#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "bindinspector.h"
#include "ui_bindinspector.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    connect(ui->label_button, &QPushButton::clicked, this, [=]() {
        //vbox->removeWidget(label);
        ui->label->deleteLater();
    });


    ivyqt = new IvyQt("IvyQtExample", "IvyQtExample Ready", this);
    ivyqt->start("127.255.255.255", 2010);

    ivyqt->bindMessage("Hello (.*)", ui->label, [=](Peer* sender, QStringList params) {
       qDebug() << "Hello with " << params[0] << " from " << sender->name();
       ui->label->setText(params[0]);
    });

    auto sendTxt = [=]() {
        ivyqt->send(ui->line_edit->text());
    };
    connect(ui->line_edit, &QLineEdit::returnPressed, this, sendTxt);
    connect(ui->send_button, &QPushButton::clicked, this, sendTxt);

    connect(ui->regex_lineEdit, &QLineEdit::returnPressed, this, &MainWindow::subscribe_regex);
    connect(ui->subscribe_button, &QPushButton::clicked, this, &MainWindow::subscribe_regex);
}

void MainWindow::subscribe_regex() {
    auto regex = ui->regex_lineEdit->text();
    qDebug() << regex;
    auto bi = new BindInspector();
    ui->sub_frame->layout()->addWidget(bi);



    auto subId = ivyqt->bindMessage(regex, bi, [=](Peer* sender, QStringList params) {
        auto txt = params.join(", ");
        bi->ui->params_label->setText(txt);
    });

    bi->ui->cbid_label->setText(QString::number(subId));

    connect(bi->ui->remove_button, &QToolButton::clicked, bi, [=]() {
        ivyqt->unBindMessage(subId);
        bi->deleteLater();
    });

}

MainWindow::~MainWindow()
{
    delete ui;
}
