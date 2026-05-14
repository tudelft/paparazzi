#ifndef LAYER_COMBO_H
#define LAYER_COMBO_H

#include <QWidget>
#include <QtWidgets>
#include "maplayercontrol.h"

class LayerCombo : public QWidget
{
    Q_OBJECT
public:
    explicit LayerCombo(QWidget *parent = nullptr);

    void addLayerControl(MapLayerControl* mlc);
    void makeLayerControl(QString name, bool initialState, int z);
    bool eventFilter(QObject *object, QEvent *event);

    void setPopo(QPoint p) {_pos = p;}

signals:
    void showLayer(QString name, bool state, int zValue, qreal opacity);
    void layerOpacityChanged(QString name, qreal opacity);
    void zValueChanged(QString name, int z);

public slots:

protected:
    virtual void mouseMoveEvent(QMouseEvent* e);
    virtual void mousePressEvent(QMouseEvent* e);
    virtual void mouseReleaseEvent(QMouseEvent* e);

private:


    QScrollArea* scroll;
    QWidget* scroll_content;
    QVBoxLayout* content_layout;
    QVBoxLayout* main_layout;

    QPoint _pos;

    QList<MapLayerControl*> map_layer_controls;

    MapLayerControl* moved_layer_control;
    QPoint press_pos;
    QLabel* moved_thumbnail;
};

#endif // LAYER_COMBO_H
