#include "waypointeditor.h"
#include "srtm_manager.h"
#include "AircraftManager.h"
#include <QDebug>
#include "coordinatestransform.h"

static const QString REF0 = "Flightplan ref0";
static const QString WGS84 = "WGS84";
static const QString WGS84_SEXA = "WGS84 sexagesimal";


WaypointEditor::WaypointEditor(WaypointItem* wi, QString ac_id, QWidget *parent) : QDialog(parent),
    wi(wi), ac_id(ac_id)
{

    setWindowTitle(QString(wi->waypoint()->getName()));

    auto lay = new QVBoxLayout(this);
    auto infoLay = new QVBoxLayout();
    lay->addLayout(infoLay);

    auto combo = new QComboBox(this);
    combo->addItem(WGS84);
    combo->addItem(WGS84_SEXA);
    combo->addItem(REF0);
    for(auto wp: AircraftManager::get()->getAircraft(ac_id)->getFlightPlan()->getWaypoints()) {
        combo->addItem(wp->getName());
    }
    infoLay->addWidget(combo);

    auto latLay = new QHBoxLayout();
    auto axis1_label = new QLabel("lat", this);
    latLay->addWidget(axis1_label);
    auto latEdit = new QLineEdit(QString::number(wi->waypoint()->getLat(), 'f', 7), this);
    latLay->addWidget(latEdit);

    auto lonLay = new QHBoxLayout();
    auto axis2_label = new QLabel("lon", this);
    lonLay->addWidget(axis2_label);
    auto lonEdit = new QLineEdit(QString::number(wi->waypoint()->getLon(), 'f', 7), this);
    lonLay->addWidget(lonEdit);

    auto altLay = new QHBoxLayout();
    altLay->addWidget(new QLabel("alt", this));
    auto altSpin = new QDoubleSpinBox(this);
    altSpin->setMinimum(-100);
    altSpin->setMaximum(5000);
    altSpin->setValue(wi->waypoint()->getAlt());
    altLay->addWidget(altSpin);

    auto aglLabel =new QLabel(this);
    altLay->addWidget(aglLabel);

    auto butLay = new QHBoxLayout();
    auto upButton = new QPushButton(this);
    auto downButton = new QPushButton(this);
    upButton->setText("+10");
    downButton->setText("-10");
    butLay->addWidget(downButton);
    butLay->addWidget(upButton);

    infoLay->addLayout(latLay);
    infoLay->addLayout(lonLay);
    infoLay->addLayout(altLay);
    infoLay->addLayout(butLay);

    auto updateAgl = [=]() {
        auto ele = SRTMManager::get()->get_elevation(wi->waypoint()->getLat(), wi->waypoint()->getLon());
        double agl;
        if(ele) {
            agl = wi->waypoint()->getAlt() - ele.value();
            aglLabel->setStyleSheet("color:black");
        } else {
            double ground_alt = AircraftManager::get()->getAircraft(wi->acId())->getFlightPlan()->getGroundAlt();
            agl = wi->waypoint()->getAlt() - ground_alt;
            aglLabel->setStyleSheet("color:red");
        }
        aglLabel->setText("AGL: " + QString::number(agl) + "m");
    };


    updateAgl();

    auto updateLatLon = [=]() {
        bool ok_a = false;
        bool ok_b = false;
        double a = latEdit->text().toDouble(&ok_a);
        double b = lonEdit->text().toDouble(&ok_b);
        if(ok_a && ok_b) {
            if(combo->currentText() == WGS84) {
                wi->waypoint()->setLat(a);
                wi->waypoint()->setLon(b);
                wi->setPosition(Point2DLatLon(a, b));
            }
            else if(combo->currentText() == WGS84_SEXA) {
                //TODO
            } else {
                Waypoint* wp;
                if(combo->currentText() == REF0) {
                    wp = wi->waypoint()->getOrigin();
                } else {
                    wp = AircraftManager::get()->getAircraft(ac_id)->getFlightPlan()->getWaypoint(combo->currentText());
                }
                Waypoint::WpFrame frame = AircraftManager::get()->getAircraft(ac_id)->getFlightPlan()->getFrame();
                wi->waypoint()->setRelative(frame, a, b, wp);
                Point2DLatLon geo(wi->waypoint());
                wi->setPosition(geo);
            }



        } else {
            qDebug() << "invalid input!!!";
        }
    };

    connect(latEdit, &QLineEdit::textEdited, [=](){
        updateLatLon();
        updateAgl();
    });

    connect(lonEdit, &QLineEdit::textEdited, [=](){
        updateLatLon();
        updateAgl();
    });


    connect(altSpin, qOverload<double>(&QDoubleSpinBox::valueChanged), [=](double alt){
        wi->waypoint()->setAlt(alt);
        updateAgl();
    });

    connect(upButton, &QAbstractButton::clicked, [=]() {
        wi->waypoint()->setAlt(wi->waypoint()->getAlt() + 10);
        altSpin->setValue(wi->waypoint()->getAlt());
        updateAgl();
    });

    connect(downButton, &QAbstractButton::clicked, [=]() {
        wi->waypoint()->setAlt(wi->waypoint()->getAlt() - 10);
        altSpin->setValue(wi->waypoint()->getAlt());
        updateAgl();
    });


    connect(combo, &QComboBox::currentTextChanged, this, [=](const QString &text) {
        if(text == WGS84) {
            axis1_label->setText("lat");
            axis2_label->setText("lon");
            latEdit->setText(QString::number(wi->waypoint()->getLat(), 'f', 7));
            lonEdit->setText(QString::number(wi->waypoint()->getLon(), 'f', 7));
        }
        else if(text == WGS84_SEXA) {
            axis1_label->setText("lat");
            axis2_label->setText("lon");
            qDebug() << "TODO sexa";
        } else {
            axis1_label->setText("x");
            axis2_label->setText("y");

            Waypoint* wp;
            if(text == REF0) {
                //wp = wi->waypoint()->getOrigin();
                wp = AircraftManager::get()->getAircraft(ac_id)->getFlightPlan()->getOrigin();
            } else {
                wp = AircraftManager::get()->getAircraft(ac_id)->getFlightPlan()->getWaypoint(text);
            }

            Waypoint::WpFrame frame = AircraftManager::get()->getAircraft(ac_id)->getFlightPlan()->getFrame();
            double x0, y0, x, y;
            if(frame == Waypoint::LTP) {
                CoordinatesTransform::get()->wgs84_to_ltp(wi->waypoint()->getOrigin(), wp, x0, y0);
                CoordinatesTransform::get()->wgs84_to_ltp(wi->waypoint()->getOrigin(), wi->waypoint(), x, y);
            } else {
                CoordinatesTransform::get()->wgs84_to_relative_utm(wi->waypoint()->getOrigin(), wp, x0, y0);
                CoordinatesTransform::get()->wgs84_to_relative_utm(wi->waypoint()->getOrigin(), wi->waypoint(), x, y);
            }
            latEdit->setText(QString::number(x-x0, 'f', 1));
            lonEdit->setText(QString::number(y-y0,  'f', 1));

        }
    });


    auto buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    lay->addWidget(buttonBox);

    connect(buttonBox->button(QDialogButtonBox::Ok), &QPushButton::clicked, this, [=](){
        emit DispatcherUi::get()->move_waypoint_ui(wi->waypoint(), ac_id);
        accept();

    });

    connect(buttonBox->button(QDialogButtonBox::Cancel), &QPushButton::clicked, this, [=](){
        reject();
    });
}
