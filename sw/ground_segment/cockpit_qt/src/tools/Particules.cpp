
#include <QApplication>
#include <QStringList>
#include "Particules.h"
#include "pprz_dispatcher.h"
#include "mapwidget.h"
#include "MainWindow.h"
#include "PprzApplication.h"
#include "pprzmain.h"

Particules* Particules::m_instance = nullptr;

Particules* Particules::get() {
    if (!m_instance) {
        m_instance = new Particules();
    }
    return m_instance;
}

Particules::Particules(QObject *parent) : QObject(parent) {
}

void Particules::init() {
    PprzDispatcher *dispatcher = PprzDispatcher::get();
    if (!dispatcher) return;
    
    // Bind to the PLUMES message
    dispatcher->bind("PLUMES", this, [this](QString sender, pprzlink::Message msg) {
        onPlumesMsg(msg);
    });
}

void Particules::onPlumesMsg(pprzlink::Message msg) {
    QString ids_str = "";
    QString lats_str = "";
    QString longs_str = "";
    QString values_str = "";
    
    try {
        msg.getField("ids", ids_str);
        msg.getField("lats", lats_str);
        msg.getField("longs", longs_str);
        msg.getField("values", values_str);
    } catch(...) {
        return; // protect against malformed PLUMES payloads crashing 
    }
    
    QStringList ids = ids_str.split(",");
    QStringList xs = lats_str.split(",");
    QStringList ys = longs_str.split(",");
    QStringList vs = values_str.split(",");
    
    // We grab the Map2D widget dynamically from the configured layout:
    MapWidget* map = pprzApp()->mainWindow()->findChild<MapWidget*>("map2d");
    if(!map) {
        // Fallback to any map widget if not named map2d perfectly
        map = pprzApp()->mainWindow()->findChild<MapWidget*>();
    }
    
    if(!map) return;

    int count = qMin(qMin(ids.size(), xs.size()), qMin(ys.size(), vs.size()));
    
    for (int i = 0; i < count; i++) {
        if (ids[i].isEmpty()) continue;
        int id = ids[i].toInt();
        double lat = xs[i].toDouble();
        double lon = ys[i].toDouble();
        int value = vs[i].toInt();
        
        Point2DLatLon wgs84(lat, lon);
        
        if (_particules.contains(id)) {
            _particules[id]->updateParticule(wgs84, value);
        } else {
            ParticuleItem* p = new ParticuleItem(id, wgs84, value, "PLUMES");
            _particules.insert(id, p);
            map->addItem(p);
        }
    }
}
