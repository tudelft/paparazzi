#ifndef MAPLAYERCONTROL_H
#define MAPLAYERCONTROL_H

#include <QWidget>
#include <QVBoxLayout>
#include <QLabel>
#include <QSlider>
#include "imagebutton.h"

class MapLayerControl : public QWidget
{
    Q_OBJECT
public:
    explicit MapLayerControl(QString name, QPixmap pixmap, bool initialState, int z, QWidget *parent = nullptr);

    void setShowState(bool state);
    void setOpacitySlider(qreal opacity);
    qreal opacity() {return static_cast<qreal>(opacitySlider->value())/opacitySlider->maximum();}
    int zValue() {return z_value;}
    void setZValue(int z);
    QPixmap pixmap() {
#if QT_VERSION_MAJOR == 5 && QT_VERSION_MINOR < 15
        QPixmap p = *imageLabel->pixmap();
#else
        QPixmap p = imageLabel->pixmap(Qt::ReturnByValue);
#endif
        return p;
    }
    QString name() {return _name;}

    bool operator <(const MapLayerControl& mlc) const
    {
        return (z_value < mlc.z_value);
    }

    //ImageButton* button() {return show_button;}

signals:
    void showLayer(bool);
    void layerOpacityChanged(qreal);
    void zValueChanged(int);


public slots:

private:
    void toggleShowState();

    QVBoxLayout* verticalLayout;
    QLabel* nameLabel;
    QLabel* imageLabel;
    QSlider* opacitySlider;
    ImageButton *show_button;

    QString _name;
    bool showState;
    int z_value;
};

#endif // MAPLAYERCONTROL_H
