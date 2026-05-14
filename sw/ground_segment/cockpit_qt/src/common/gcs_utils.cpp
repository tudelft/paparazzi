#include "gcs_utils.h"
#include <QRegularExpression>
#include <QSettings>
#include <QtXml>

QString user_or_app_path(QString data_path) {
    auto user_path = appConfig()->value("USER_DATA_PATH").toString();
    auto app_path = appConfig()->value("APP_DATA_PATH").toString();
    
    auto path = user_path + "/" + data_path;
    if(QFile::exists(path)) {
        return path;
    } else {
        return app_path + "/" + data_path;
    }
}

double getFloatingField(pprzlink::Message &msg, const QString &field) {
    try {
        double val;
        msg.getField(field, val);
        return val;
    }  catch (std::bad_any_cast& e) {
        float val;
        msg.getField(field, val);
        return static_cast<double>(val);
    }
}

double parse_coordinate(QString str) {
    str = str.toUpper();

    QRegularExpression decimal_rx(R"([+-]?([0-9]*[.])?[0-9]+)");
    QRegularExpression sexa_rx(R"(([+-]?[0-9]+) ([0-9]+) ((?:[0-9]*[.])?[0-9]+) ?([NSEW]?))");

    auto decimal_match = decimal_rx.match(str);
    if(decimal_match.hasMatch() && decimal_match.capturedLength() == str.length()) {
        return str.toDouble();
    } 

    auto sexa_match = sexa_rx.match(str);
    if(sexa_match.hasMatch()) {
        auto caps = sexa_match.capturedTexts();
        if(caps.length() == 5) {
            auto deg = caps[1].toInt();
            auto min = caps[2].toInt();
            auto sec = caps[3].toDouble();
            auto dir = caps[4];
            if(dir == "S" || dir == "W") {
                deg = -deg;
                min = -min;
                sec = -sec;
            }
            double coor = deg + min / 60.0 + sec / 3600.0;
            return coor;
        }
    }
    throw std::runtime_error(str.toStdString() + " is not a coordinate");
}

QSettings getAppSettings() {
    return QSettings(appConfig()->value("SETTINGS_PATH").toString(), QSettings::IniFormat);
}

GlobalConfig* appConfig() {
    return GlobalConfig::get();
}

bool verbose() {
    return qApp->property("VERBOSE").toBool();
}

void setVerbose(bool v) {
    qApp->setProperty("VERBOSE", v);
}

bool speech() {
    return qApp->property("SPEECH").toBool();
}

void setSpeech(bool s) {
    qApp->setProperty("SPEECH", s);
}

void logDebug(QString log_class, QString msg, LogLevel level) {
    (void)level;
    if(qApp->property("VERBOSE").toBool()) {
        auto str = QString("[%1] %2").arg(log_class, msg);
        qDebug() << str;
    }
}


QStringList getLayoutFiles(QString location) {
    auto path = appConfig()->value(location).toString();
    if(path.isNull()) {
        return QStringList();
    }

    QDir conf_dir(path);
    auto xml_files = conf_dir.entryList({"*.xml"});
    QStringList layout_files;
    for(auto &fn:xml_files) {
        QFile f(path + "/" + fn);
        if(f.open(QIODevice::ReadOnly)) {
            QDomDocument doc;
            doc.setContent(&f);
            auto ele = doc.firstChildElement( "gcsconf" );
            if(!ele.isNull()) {
                layout_files.append(fn);
            }
            f.close();
        }
    }
    return layout_files;
}
