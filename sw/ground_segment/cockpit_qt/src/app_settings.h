#ifndef CONFIGURE_H
#define CONFIGURE_H

#include <QString>
#include <QDialog>
#include <QtWidgets>

void set_app_settings();

class SettingsEditor : public QDialog
{
    Q_OBJECT
public:
    explicit SettingsEditor(bool standalone = false, QWidget *parent = nullptr);



private:

    enum Type {
        STRING,
        INT,
        DOUBLE,
        PATH_DIR,
        PATH_FILE,
        STRING_COMBO,
    };

    std::function<void()> addSetting(QString name, QString key, QWidget* w, QGridLayout* gl, int &row, Type type, QVariant data = QVariant());

    QList<std::function<void()>> callbacks;

};

#endif // CONFIGURE_H
