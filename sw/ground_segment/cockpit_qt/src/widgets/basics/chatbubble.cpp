#include "chatbubble.h"
#include "ui_chatbubble.h"
#include "AircraftManager.h"
#include <QDebug>

ChatBubble::ChatBubble(QString source, QString dest, QString msg, bool sent, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ChatBubble),
    source(source),
    dest(dest)
{
    ui->setupUi(this);

    QString source_name = source;
    QColor color = palette().color(QPalette::WindowText);

    if(AircraftManager::get()->aircraftExists(source)) {
        auto ac = AircraftManager::get()->getAircraft(source);
        source_name = ac->name();
        color = ac->getColor();
    }
    if(dest != "") {
        source_name += QString::fromUtf8(" \u2794 ") + dest;
    }

    // Add option for bold text
    msg = msg.toHtmlEscaped();
    QRegularExpression re(QStringLiteral("\\*([^*]+)\\*"));
    msg.replace(re, "<b>\\1</b>");

    chat_time = QTime::currentTime();
    auto date = chat_time.toString();
    ui->time_label->setText(date+" ");
    ui->name_label->setText("<b>"+source_name+"</b>");
    ui->name_label->setStyleSheet("color:" + color.name());
    ui->message_label->setText(msg);
    ui->message_label->setTextFormat(Qt::RichText);

    if(sent) {
        ui->name_label->setAlignment(Qt::AlignLeft);
        ui->horizontalLayout->setAlignment(Qt::AlignLeft);
    } else {
        ui->name_label->setAlignment(Qt::AlignRight);
        ui->horizontalLayout->setAlignment(Qt::AlignRight);
    }

    // delay set max width to wait for the widget to be shown.
    QTimer::singleShot(20, this, [=](){
            auto width = size().width() * 0.95;
            ui->frame->setMaximumWidth(width);
    });
}

ChatBubble::~ChatBubble()
{
    delete ui;
}

void ChatBubble::addText(QString text) {
    // Add option for bold text
    text = text.toHtmlEscaped();
    QRegularExpression re(QStringLiteral("\\*([^*]+)\\*"));
    text.replace(re, "<b>\\1</b>");

    chat_time = QTime::currentTime();
    ui->message_label->setText(ui->message_label->text() + "<br>" + text);
    ui->message_label->setTextFormat(Qt::RichText);
}

bool ChatBubble::eventFilter(QObject *obj, QEvent *ev) {
    (void)obj;
    (void)ev;
    if(ev->type() == QEvent::Resize) {
        auto width = size().width() * 0.95;
        ui->frame->setMaximumWidth(width);
    }

    return false;
}
