#include "chat.h"
#include "ui_chat.h"
#include "pprz_dispatcher.h"
#include "gcs_utils.h"
#include "AircraftManager.h"
#include "dispatcher_ui.h"
#include "chatbubble.h"
#include <QtWidgets>
#include <QDebug>
#include <ostream>

Chat::Chat(QWidget *parent) :
    QFrame(parent),
    ui(new Ui::Chat)
{
    ui->setupUi(this);

    auto settings_path = appConfig()->value("SETTINGS_PATH").toString();
    QSettings settings(settings_path, QSettings::IniFormat);
    chat_id = settings.value("pprzlink/id").toString();

    connect(DispatcherUi::get(), &DispatcherUi::new_ac_config, this,  &Chat::onNewAircraft);
    connect(DispatcherUi::get(), &DispatcherUi::ac_deleted, this, &Chat::onAcDeleted);

    PprzDispatcher::get()->bind("INFO_MSG", this, [=](QString sender, pprzlink::Message msg) {
        QByteArray ar;
        msg.getField("msg", ar);
        auto txt = QString(ar);
        addMessage(txt, sender);
    });

    try {
        PprzDispatcher::get()->bind("INFO_MSG_GROUND", this, [=](QString sender, pprzlink::Message msg) {
            (void) sender;
            QString txt;
            QString source;
            QString dest;
            msg.getField("msg", txt);
            msg.getField("source", source);
            msg.getField("dest", dest);
            auto dest_splitted = dest.split(':');
            QString chat_dst;

            if(dest_splitted[0] == chat_id) {
                chat_dst = chat_id;
            } else {
                bool isInt;
                int dest_id = dest_splitted[0].toInt(&isInt, 0);
                if(isInt) {
                    if(dest_id == 0xFF) {
                        chat_dst = "All";
                    } else if(dest_id == 0) {
                        chat_dst = "Ground";
                    }
                }
            }
            if(!chat_dst.isNull()) {
                addMessage(txt, source, chat_dst);
            }
        });

        connect(ui->msg_lineedit, &QLineEdit::returnPressed, this, &Chat::onSend);
        connect(ui->send_button, &QPushButton::clicked, this, &Chat::onSend);

    } catch (pprzlink::no_such_message &e) {
        qDebug() << e.what();
        ui->msg_lineedit->setDisabled(true);
        ui->send_button->setDisabled(true);
        ui->fd_combo->setDisabled(true);
        ui->name_combo->setDisabled(true);
        ui->msg_lineedit->setPlaceholderText("Update paparazzi to send messages");
    }
    setAutoFillBackground(true);
}

Chat::~Chat()
{
    delete ui;
}

void Chat::onNewAircraft(QString ac_id) {
    auto ac = AircraftManager::get()->getAircraft(ac_id);
    ui->name_combo->addItem(ac->name());
}

void Chat::onAcDeleted(QString ac_id) {
    auto ac = AircraftManager::get()->getAircraft(ac_id);
    auto idx = ui->name_combo->findText(ac->name());
    if(idx != -1) {
        ui->name_combo->removeItem(idx);
    }
}

void Chat::onSend() {
    auto text = ui->msg_lineedit->text();
    auto fd = ui->fd_combo->currentIndex();
    auto name = ui->name_combo->currentText();
    QString dest;

    if(name == "All") {
        dest = "0xFF";
    } else {
        dest = AircraftManager::get()->getAircraftByName(name).value()->getId();
    }
    dest += ":" + QString::number(fd);

    auto msgDef = PprzDispatcher::get()->getDict()->getDefinition("INFO_MSG_GROUND");
    pprzlink::Message pprz_msg(msgDef);
    pprz_msg.addField("dest", dest);
    pprz_msg.addField("source", chat_id);
    pprz_msg.addField("msg", text);
    PprzDispatcher::get()->sendMessage(pprz_msg);
    addMessage(text, "me", name, true);

    ui->msg_lineedit->clear();
}


void Chat::addMessage(QString txt, QString source, QString dst, bool sent) {
    (void)dst;
    // Check if it can be combined with the previous message
    if(ui->scroll_widget->layout()->count() > 0) {
        auto last = ui->scroll_widget->layout()->itemAt(ui->scroll_widget->layout()->count()-1);
        auto cb = dynamic_cast<ChatBubble*>(last->widget());
        if(cb != nullptr && cb->getSource() == source && cb->getDest() == dst && cb->getTime().secsTo(QTime::currentTime()) < 5){
            cb->addText(txt);
            return;
        }
    }

    auto cb = new ChatBubble(source, dst, txt, sent, this);
    ui->scroll_widget->layout()->addWidget(cb);
    ui->scroll_widget->installEventFilter(cb);

    // delay scroll to bottom to wait for the x=widget to be repainted.
    QTimer::singleShot(20, this, [=](){
        ui->scrollArea->verticalScrollBar()->setSliderPosition(ui->scrollArea->verticalScrollBar()->maximum());
    });

}
