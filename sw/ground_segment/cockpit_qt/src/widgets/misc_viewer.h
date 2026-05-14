#ifndef MISCVIEWER_H
#define MISCVIEWER_H

#include <QWidget>
#include <QLabel>
#include <QGridLayout>
#include <QCheckBox>
#include <QMap>

class MiscViewer : public QWidget
{
    Q_OBJECT
public:
    explicit MiscViewer(QString ac_id, QWidget *parent = nullptr);
    void setValue(const QString& label, const QString& text);

private:
    QString ac_id;

    QGridLayout* table;
    QMap<QString, QLabel*> values;
    QCheckBox* periodic_send;
};

#endif // MISCVIEWER_H
