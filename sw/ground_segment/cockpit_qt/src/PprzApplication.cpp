#include "PprzApplication.h"
#include "PprzToolbox.h"
#include "pprzmain.h"
#include "point2dlatlon.h"

PprzApplication* PprzApplication::_app = nullptr;

PprzApplication::PprzApplication(int &argc, char **argv) : QApplication(argc, argv)
{
    _app = this;
    _toolbox = new PprzToolbox(this);
    //_toolbox->setChildToolboxes();
    _mainWindow = new PprzMain();
    qRegisterMetaType<Point2DLatLon>();
}

/// @brief Returns the PprzApplication object singleton.
PprzApplication* pprzApp(void)
{
    return PprzApplication::_app;
}

void PprzApplication::init() {
    _toolbox->setChildToolboxes();
}

void PprzApplication::_shutdown() {
    delete _toolbox;
    delete _mainWindow;
}

PprzApplication::~PprzApplication() {
    _app = nullptr;
}
