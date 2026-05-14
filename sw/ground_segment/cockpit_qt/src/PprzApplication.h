#ifndef PPRZAPPLICATION_H
#define PPRZAPPLICATION_H

#include <QApplication>

class PprzToolbox;
class PprzMain;

class PprzApplication : public QApplication
{
    Q_OBJECT
public:
    explicit PprzApplication(int &argc, char **argv);
    ~PprzApplication();

    static PprzApplication*  _app;
    void init();

    void _shutdown();

    PprzToolbox* toolbox() {return _toolbox;}
    PprzMain* mainWindow() {return _mainWindow;}

private:
    PprzToolbox* _toolbox = nullptr;
    PprzMain* _mainWindow = nullptr;

signals:

};

/// @brief Returns the PprzApplication object singleton.
PprzApplication* pprzApp(void);

#endif // PPRZAPPLICATION_H
