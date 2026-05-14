#pragma once
#include <QString>
#include <QApplication>
#include <QFile>
#include <QSettings>
#include "globalconfig.h"
#include <pprzlinkQt/Message.h>

enum LogLevel {
    LOG_LOW,
    LOG_MID,
    LOG_HIGH,
};

QString user_or_app_path(QString data_path);

double parse_coordinate(QString str);

double getFloatingField(pprzlink::Message &msg, const QString &field);

QSettings getAppSettings();
GlobalConfig* appConfig();

bool verbose();
void setVerbose(bool v);
bool speech();
void setSpeech(bool s);

void logDebug(QString log_class, QString msg, LogLevel level = LogLevel::LOG_MID);
QStringList getLayoutFiles(QString location);
