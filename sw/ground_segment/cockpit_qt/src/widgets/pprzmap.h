#ifndef PPRZMAP_H
#define PPRZMAP_H

#include <QWidget>
#include <QKeyEvent>
#include "map_item.h"
#include "item_edit_state_machine.h"
#include "aircraft_item.h"
#include <pprzlinkQt/Message.h>
#include "coordinatestransform.h"
#include "QScrollArea"
#include "layer_combo.h"
#include "maplayercontrol.h"
#include "configurable.h"
#include "acitemmanager.h"

namespace Ui {
class PprzMap;
}



class PprzMap : public QWidget, public Configurable
{
    Q_OBJECT

public:
    explicit PprzMap(QWidget *parent = nullptr);
    MapLayerControl* makeLayerControl(QString name, bool initialState, int z);
    void configure(QDomElement e);

protected:
//    void scenePressEvent(QGraphicsSceneMouseEvent *mouseEvent);
//    void sceneMoveEvent(QGraphicsSceneMouseEvent *mouseEvent);
//    void sceneReleaseEvent(QGraphicsSceneMouseEvent *mouseEvent);


protected slots:
    void handleMouseMove(QPointF scenePos);

private slots:
    void changeCurrentAC(QString id);

private:
    void setEditorMode();
    Ui::PprzMap *ui;
    QMap<Waypoint*, int> combo_indexes;

    QString current_ac;

    //CoordinatesTransform ct_wgs84_utm;
};

#endif // PPRZMAP_H
