#include <QApplication>
#include "PlotterWindow.h"

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    
    app.setApplicationName("Paparazzi Plotter Qt");
    app.setApplicationVersion("1.0");

    PlotterWindow window;
    window.resize(800, 600);
    window.show();

    return app.exec();
}