#ifndef UNITS_H
#define UNITS_H
#include <optional>
#include <QtCore>
#include "PprzToolbox.h"
#include "PprzApplication.h"

class Units : public PprzTool {
public:
    explicit Units(PprzApplication* app, PprzToolbox* toolbox);
    static Units* get() {
        return pprzApp()->toolbox()->units();
    }

    virtual void setToolbox(PprzToolbox* toolbox) override;

    std::optional<double> getCoef(QString inputUnit, QString outputUnit);
    double convert(double value, QString from, QString to);

private:
    QMap<QPair<QString, QString>, double> coefs;
};
#endif // UNITS_H
