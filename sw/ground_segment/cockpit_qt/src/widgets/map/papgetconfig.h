#ifndef PAPGETCONFIG_H
#define PAPGETCONFIG_H

#include <QtWidgets>
#include "papget.h"

class PapgetConfig : public QDialog
{
    Q_OBJECT
public:
    explicit PapgetConfig(Papget::DataDef datadef, Papget::Params params, QWidget *parent = nullptr);

signals:
    void paramsChanged(Papget::Params);

public slots:

private:
    Papget::DataDef datadef;
    QList<std::function<void()>> callbacks;
    int current_index;

    std::function<void()> config_text(QWidget* w);

    Papget::Params params;

};


#endif // PAPGETCONFIG_H
