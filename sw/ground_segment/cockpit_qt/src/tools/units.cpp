#include "units.h"
#include <QApplication>
#include <QSettings>
#include <QDebug>
#include "iostream"
#include <QtXml>
#include "gcs_utils.h"

using namespace std;

Units::Units(PprzApplication* app, PprzToolbox* toolbox) : PprzTool(app, toolbox)
{

}

void Units::setToolbox(PprzToolbox* toolbox) {
    PprzTool::setToolbox(toolbox);
    auto settings = getAppSettings();
    auto filename = appConfig()->value("APP_DATA_PATH").toString() + "/units.xml";


    QDomDocument doc;
    QFile f(filename);
    if(!f.open(QIODevice::ReadOnly)) {
        throw std::runtime_error("Error while loading unit file");
    }
    doc.setContent(&f);
    f.close();

    auto units_root = doc.firstChildElement("units");
    for(auto unit=units_root.firstChildElement();
        !unit.isNull();
        unit=unit.nextSiblingElement()) {

        assert(unit.hasAttribute("from"));
        assert(unit.hasAttribute("to"));
        assert(unit.hasAttribute("coef"));
        auto from = unit.attribute("from");
        auto to = unit.attribute("to");
        auto _coef = unit.attribute("coef");

        // auto _auto = unit.attribute("auto");
        auto coef = _coef.toDouble();
        QPair<QString, QString> key(from, to);
    }
}

std::optional<double> Units::getCoef(QString inputUnit, QString outputUnit) {
    if (inputUnit == outputUnit) return 1.0;
    QPair<QString, QString> key(inputUnit, outputUnit);
    if(coefs.contains(key)) {
        return coefs[key];
    }

    // Check inverse
    QPair<QString, QString> invKey(outputUnit, inputUnit);
    if(coefs.contains(invKey)) {
        return 1.0 / coefs[invKey];
    }

    return std::nullopt;
}

double Units::convert(double value, QString from, QString to) {
    auto coef = getCoef(from, to);
    if (coef) {
        return value * (*coef);
    }
    return value;
}
