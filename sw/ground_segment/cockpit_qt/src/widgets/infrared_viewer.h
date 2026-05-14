#ifndef INFRAREDVIEWER_H
#define INFRAREDVIEWER_H

#include <QWidget>
#include <QLabel>
#include <QGridLayout>
#include <QProgressBar>

class InfraredViewer : public QWidget
{
    Q_OBJECT
public:
    explicit InfraredViewer(QString ac_id, QWidget *parent = nullptr);

public slots:
    void setContrastStatus(const QString& s);
    void setContrastValue(int s);
    void setGpsHybridMode(const QString& s);
    void setGpsHybridFactor(float s);
    void updateData();

private:
    QString ac_id;

    QLabel* contrast_status;
    QLabel* contrast_value;
    QLabel* gps_hybrid_mode;
    QLabel* gps_hybrid_factor;

    QProgressBar* ir_left;
    QProgressBar* ir_front;
    QProgressBar* ir_top;

    QProgressBar* roll_bar;
    QProgressBar* pitch_bar;
};

#endif // INFRAREDVIEWER_H
