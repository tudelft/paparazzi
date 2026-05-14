#ifndef PPRZMAIN_H
#define PPRZMAIN_H

#include <QMainWindow>
#include <QtWidgets>

enum LaunchTypes {
    DEFAULT,
    NORMAL,
    FLIGHTPLAN_EDIT,
    CONFIGURE,
    QUIT,
};

class PprzMain : public QMainWindow
{
    Q_OBJECT

public:
    explicit PprzMain(QWidget *parent = nullptr);
    static LaunchTypes launch_type;
    static int const EXIT_CODE_REBOOT = -123456;
    void setupUi(int width, int height, QWidget* centralWidget);

    void setServerStatus(bool active);

signals:
    void ready();

protected:
    void closeEvent(QCloseEvent *event) override;

private:
    void newAC(QString ac_id);
    void removeAC(QString ac_id);
    void delete_ac(QString ac_id);

    void populate_menu();

    QMenuBar *menu_bar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    QMenu* aircraftsTopMenu;
    QMap<QString, QMenu*> acMenus;
    QMap<QString, QAction*> acActions;
    QLabel* serverStatusLed;
};


#endif // PPRZMAIN_H
