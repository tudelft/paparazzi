#include "Live.h"
#include "pprz_dispatcher.h"

Live* Live::m_instance = nullptr;

Live* Live::get() {
    if (!m_instance) {
        m_instance = new Live();
    }
    return m_instance;
}

Live::Live(QObject *parent) : QObject(parent) {
}

void Live::init() {
    // Porting the message bindings from live.ml
    // In live.ml, multiple basic bindings are created to intercept flight_param, ap_status, etc.
    
    PprzDispatcher *dispatcher = PprzDispatcher::get();
    if (!dispatcher) return;
    
    // As in live.ml we would dispatch messages like ALIVE, FLIGHT_PARAM, etc.
    // The pprz_dispatcher in C++ already handles routing but we will connect the live signal processing.
    
    connect(dispatcher, &PprzDispatcher::flight_param, this, [this](pprzlink::Message msg){
        QString ac_id = msg.getDefinition().getName();
        emit messageReceived(ac_id, "FLIGHT_PARAM");
    });
    
    connect(dispatcher, &PprzDispatcher::ap_status, this, [this](pprzlink::Message msg){
        emit messageReceived(msg.getDefinition().getName(), "AP_STATUS");
    });
    
    connect(dispatcher, &PprzDispatcher::telemetry_status, this, [this](pprzlink::Message msg){
        emit messageReceived(msg.getDefinition().getName(), "TELEMETRY_STATUS");
    });
    
    connect(dispatcher, &PprzDispatcher::engine_status, this, [this](pprzlink::Message msg){
        emit messageReceived(msg.getDefinition().getName(), "ENGINE_STATUS");
    });

}

void Live::onPprzMessage() {
    // Basic slot placeholder for future direct binds, though lambda connects handle it fine.
}
