#include "Intruders.h"
#include "pprz_dispatcher.h"
#include "mapwidget.h"
#include "MainWindow.h"
#include "PprzApplication.h"
#include "pprzmain.h"
#include "intruder_item.h"
#include <QDateTime>

Intruders* Intruders::m_instance = nullptr;

Intruders* Intruders::get() {
    if (!m_instance) {
        m_instance = new Intruders();
    }
    return m_instance;
}

Intruders::Intruders(QObject *parent) : QObject(parent) {
    _timer = new QTimer(this);
    connect(_timer, &QTimer::timeout, this, &Intruders::removeOldIntruders);
    _timer->start(1000); // Check every second
}

void Intruders::init() {
    PprzDispatcher *dispatcher = PprzDispatcher::get();
    if (!dispatcher) return;
    
    dispatcher->bind("INTRUDER", this, [this](QString sender, pprzlink::Message msg) {
        onIntruderMsg(sender, msg);
    });
}

void Intruders::onIntruderMsg(QString sender, pprzlink::Message msg) {
    (void)sender;
    
    QString id;
    QString name;
    int32_t lat_int, lon_int;
    float course;
    float speed = 0.0f;
    float climb = 0.0f;
    int32_t alt_int = 0;
    
    try {
        msg.getField("id", id);
        msg.getField("name", name);
        msg.getField("lat", lat_int);
        msg.getField("lon", lon_int);
        msg.getField("course", course);
        msg.getField("speed", speed);
        msg.getField("climb", climb);
        msg.getField("alt", alt_int);
    } catch(...) {
        return; // Ignore malformed messages
    }
    
    double lat = lat_int / 1e7;
    double lon = lon_int / 1e7;
    double alt = alt_int / 1000.0;
    Point2DLatLon wgs84(lat, lon);
    
    double current_time = QDateTime::currentMSecsSinceEpoch() / 1000.0;
    
    if (_intruders.contains(id)) {
        // Update existing
        Intruder& intruder = _intruders[id];
        if (intruder.track) {
            intruder.track->setPosition(wgs84);
            intruder.track->setCourse(course);
            intruder.track->setAltitude(alt);
            intruder.track->setSpeed(speed);
            intruder.track->setClimb(climb);
        }
        intruder.last_update = current_time;
    } else {
        // Create new
        MapWidget* map = nullptr;
        if (pprzApp() && pprzApp()->mainWindow()) {
            map = pprzApp()->mainWindow()->findChild<MapWidget*>("map2d");
            if(!map) map = pprzApp()->mainWindow()->findChild<MapWidget*>();
        }
        if(!map) return;
        
        IntruderItem* track = new IntruderItem(name, wgs84, course);
        track->setAltitude(alt);
        track->setSpeed(speed);
        track->setClimb(climb);
        map->addItem(track);
        
        Intruder intruder;
        intruder.track = track;
        intruder.last_update = current_time;
        _intruders.insert(id, intruder);
    }
}

void Intruders::removeOldIntruders() {
    double current_time = QDateTime::currentMSecsSinceEpoch() / 1000.0;
    
    MapWidget* map = nullptr;
    if (pprzApp() && pprzApp()->mainWindow()) {
        map = pprzApp()->mainWindow()->findChild<MapWidget*>("map2d");
        if(!map) map = pprzApp()->mainWindow()->findChild<MapWidget*>();
    }
    
    auto it = _intruders.begin();
    while (it != _intruders.end()) {
        if (current_time - it.value().last_update > 20.0) {
            if (it.value().track) {
                if (map) {
                    map->removeItem(it.value().track);
                } else {
                    // If map is already dead or not found, just delete safely
                    it.value().track->deleteLater();
                }
            }
            it = _intruders.erase(it);
        } else {
            ++it;
        }
    }
}
