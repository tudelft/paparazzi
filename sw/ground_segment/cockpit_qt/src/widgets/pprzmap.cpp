#include <utility>
#include "pprzmap.h"
#include "ui_pprzmap.h"
#include "pprz_dispatcher.h"
#include "dispatcher_ui.h"
#include <iostream>
#include "maputils.h"
#include "AircraftManager.h"
#include "pprzmain.h"
#include "srtm_manager.h"
#include "point2dpseudomercator.h"
#include <QSet>
#include "gcs_utils.h"
#if GRPC_ENABLED
#include "grpcconnector.h"
#endif

PprzMap::PprzMap(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PprzMap),
    current_ac("0")
{
    ui->setupUi(this);

    auto dl_strm = [=](bool local_only) {

        QSet<QString> tiles;

        // Download SRTM tile(s) for each flightplan footprint
        for(auto ac: AircraftManager::get()->getAircrafts()) {
            auto [nw, se] = ac->getFlightPlan()->boundingBox();
            auto tile_names = SRTMManager::get()->get_tile_names(se.lat(), nw.lat(), nw.lon(), se.lon());
#if QT_VERSION_MAJOR == 5 && QT_VERSION_MINOR < 15
            tiles.unite(tile_names.toSet());
#else
            auto tiles_set = QSet<QString>(tile_names.begin(), tile_names.end());
            tiles.unite(tiles_set);
#endif
        }

        // Download SRTM tile(s) for view footprint
        Point2DLatLon nw(0, 0), se(0, 0);
        ui->map->getViewPoints(nw, se);
        auto tile_names = SRTMManager::get()->get_tile_names(se.lat(), nw.lat(), nw.lon(), se.lon());
#if QT_VERSION_MAJOR == 5 && QT_VERSION_MINOR < 15
        tiles.unite(tile_names.toSet());
#else
        auto tiles_set = QSet<QString>(tile_names.begin(), tile_names.end());
        tiles.unite(tiles_set);
#endif
        SRTMManager::get()->load_tiles(tiles.values(), local_only);

    };

    connect(ui->srtm_button, &QPushButton::clicked, [=]() {dl_strm(false);});
#if GRPC_ENABLED
    connect(GRPCConnector::get(), &GRPCConnector::dl_srtm, [=]() {dl_strm(true);});
#endif

    connect(DispatcherUi::get(), &DispatcherUi::ac_selected, this, &PprzMap::changeCurrentAC);
    connect(ui->map, &MapWidget::mouseMoved, this, &PprzMap::handleMouseMove);

    connect(AircraftManager::get(), &AircraftManager::waypoint_changed, this,
            [=](Waypoint* wp, QString ac_id) {
        (void)wp;
        if(current_ac == ac_id) {
            if(combo_indexes.contains(wp)) {
                ui->reference_combobox->setItemText(combo_indexes[wp], wp->getName());
            }
        }
    });
    connect(AircraftManager::get(), &AircraftManager::waypoint_added, this,
            [=](Waypoint* wp, QString ac_id) {
        (void)wp;
        if(current_ac == ac_id) {
            changeCurrentAC(ac_id);
        }
    });

}

void PprzMap::changeCurrentAC(QString id) {
    if(AircraftManager::get()->aircraftExists(id)) {

        QString current_txt= ui->reference_combobox->currentText();

        current_ac = id;
        auto waypoints = AircraftManager::get()->getAircraft(id)->getFlightPlan()->getWaypoints();

        // remove previous waypoints (the fist 2 items are WGS84 reference)
        while(ui->reference_combobox->count() > 2) {
            ui->reference_combobox->removeItem(2);
        }
        combo_indexes.clear();

        for(auto &wp: waypoints) {
            auto wp_name = wp->getName();
            if(wp_name[0] != '_') {
                ui->reference_combobox->addItem(wp_name);
                combo_indexes[wp] = ui->reference_combobox->count()-1;
            }
        }

        ui->reference_combobox->setCurrentText(current_txt);
    }
}

void PprzMap::configure(QDomElement e) {
    ui->map->configure(e);
}

void PprzMap::handleMouseMove(QPointF scenePos) {
    auto tp = tilePoint(scenePos, zoomLevel(ui->map->zoom()), ui->map->tileSize());
    Point2DPseudoMercator ppm(tp);
    auto pt = CoordinatesTransform::get()->pseudoMercator_to_WGS84(ppm);

    if(ui->reference_combobox->currentIndex() == 0) {
        auto txt = pt.toString();
        ui->pos_label->setText(txt);
    } else if (ui->reference_combobox->currentIndex() == 1) {
        auto txt = pt.toString(true);
        ui->pos_label->setText(txt);
    } else {
        auto wps = AircraftManager::get()->getAircraft(current_ac)->getFlightPlan()->getWaypoints();
        for(auto ref_wp: std::as_const(wps)) {
            if(ref_wp->getName() == ui->reference_combobox->currentText()) {
                Point2DLatLon pt_wp(ref_wp);
                double distance, azimut;
                CoordinatesTransform::get()->distance_azimut(pt_wp, pt, distance, azimut);
                ui->pos_label->setText(QString("%1").arg(static_cast<int>(azimut), 3, 10, QChar(' ')) + "° " +
                                       QString("%1").arg(static_cast<int>(distance), 4, 10, QChar(' ')) + "m");
                break;
            }
        }
    }

    auto ele = SRTMManager::get()->get_elevation(pt.lat(), pt.lon());
    if(ele) {
        QString txt = QString("%1").arg(ele.value(), 4, 10, QChar(' ')) + "m";
        ui->srtm_label->setText(txt);
    } else {
        ui->srtm_label->setText("No SRTM");
    }
}

