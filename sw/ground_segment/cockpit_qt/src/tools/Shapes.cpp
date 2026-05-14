#include "Shapes.h"
#include <QDebug>
#include <QStringList>
#include <QApplication>
#include "AircraftManager.h"
#include "mapwidget.h"
#include "MainWindow.h"

Shapes* Shapes::singleton = nullptr;

Shapes* Shapes::get() {
    if(singleton == nullptr) {
        singleton = new Shapes();
    }
    return singleton;
}

Shapes::Shapes(QObject *parent) : QObject(parent) {
}

void Shapes::init() {
    PprzDispatcher::get()->bind("SHAPE", this, [this](QString sender, pprzlink::Message msg){
        handleShapeMessage(sender, msg);
    });
}

void Shapes::handleShapeMessage(QString sender, pprzlink::Message msg) {
    uint8_t id = 0;
    msg.getField("id", id);
    
    uint8_t status = 0;
    msg.getField("status", status); // 0=Update, 1=Delete
    
    MapWidget* map = nullptr;
    foreach(QWidget *widget, QApplication::topLevelWidgets()) {
        if(MainWindow *mainWin = qobject_cast<MainWindow*>(widget)) {
            map = mainWin->findChild<MapWidget*>();
            break;
        }
    }
    if(!map) return;
    
    uint8_t shape_type_val = 0;
    msg.getField("shape", shape_type_val);
    ShapeType type = (ShapeType)shape_type_val;

    int unique_id = (shape_type_val << 8) | id;

    if (status == 1) { // Delete
        if (_shapes.contains(unique_id)) {
            auto* item = _shapes.take(unique_id);
            item->removeFromScene(map);
            delete item;
        }
        return;
    }
    
    QString lineColorStr = "";
    msg.getField("linecolor", lineColorStr);
    
    QString fillColorStr = "";
    msg.getField("fillcolor", fillColorStr);
    
    uint8_t opacity = 255;
    msg.getField("opacity", opacity);
    int alpha = 255;
    switch(opacity) {
        case 0: alpha = 0; break;
        case 1: alpha = 85; break;
        case 2: alpha = 170; break;
        case 3: alpha = 255; break;
        default: alpha = opacity; break; // fallback in case it was explicitly set via some legacy script
    }
    
    float radius = 0;
    msg.getField("radius", radius);
    
    QString textStr = "";
    msg.getField("text", textStr);
    
    QList<int32_t> latarr;
    QList<int32_t> lonarr;
    msg.getField("latarr", latarr);
    msg.getField("lonarr", lonarr);
    
    if(latarr.size() != lonarr.size()) {
        qDebug() << "Shapes: latarr and lonarr have different size. Abort.";
        return;
    }
    
    QList<Point2DLatLon> pts;
    for(int i=0; i<latarr.size(); ++i) {
        pts.append(Point2DLatLon(latarr[i]/1e7, lonarr[i]/1e7));
    }
    
    if (pts.isEmpty()) return;
    
    QColor lineColor = QColor(lineColorStr);
    QColor fillColor = QColor(fillColorStr);
    
    if (!_shapes.contains(unique_id)) {
        _shapes[unique_id] = new ShapeItem(id, type, "SHAPE");
        map->addItem(_shapes[unique_id]);
        _shapes[unique_id]->addToMap(map);
    }
    
    _shapes[unique_id]->updateShape(pts, radius, textStr, lineColor, fillColor, alpha);
}
