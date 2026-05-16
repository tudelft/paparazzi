#include <QApplication>
#include <QStyleFactory>
#include "MainWindow.h"
#include <cstdlib>

int main(int argc, char *argv[])
{
    // Force GTK3 platform theme which natively supports Ubuntu Adwaita dark/light
    qputenv("QT_QPA_PLATFORMTHEME", "gtk3");
    
    QApplication a(argc, argv);
    
    // Attempting to load native platform theme gracefully
    a.setDesktopSettingsAware(true);
    
    MainWindow w;
    w.show();
    return a.exec();
}
