#ifndef GVF_SETTINGS_H
#define GVF_SETTINGS_H

#include <QWidget>
#include <QtWidgets>
#include <QBoxLayout>
#include <QMouseEvent>
#include "colorbar.h"

class GVFViewer : public QWidget
{
    Q_OBJECT
public:
    explicit GVFViewer(QString ac_id, QWidget *parent = nullptr);

private:
    void init();

    QString ac_id;

    QVector<int> gvfV_config;
    QVector<int> gvfV_default_Vfield_config;
    QVector<float> gvfV_parametric_config;

    QString viewer_mode;
};

#endif // GVF_SETTINGS_H