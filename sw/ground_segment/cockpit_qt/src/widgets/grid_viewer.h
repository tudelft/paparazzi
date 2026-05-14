#pragma once

#include <QWidget>
#include <QString>

class GridViewer : public QWidget {
    Q_OBJECT
public:
    explicit GridViewer(QString ac_id, QWidget *parent = nullptr);
private:
    QString ac_id;
};
