#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QScrollArea>
#include <QVBoxLayout>
#include "widgets/strip.h"
#include "widgets/horizon_viewer.h"

class MapWidget;
class QLabel;
class QDoubleSpinBox;
class QTabWidget;

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

public slots:
    void addAC(QString ac_id);
    void removeAC(QString ac_id);
    void handleMouseMove(QPointF scenePos);
    void handleACSelected(QString ac_id);

private:
    QWidget *m_stripContainer;
    QVBoxLayout *m_stripLayout;
    MapWidget *m_map;
    
    QTabWidget *m_infoTabs;
    QMap<QString, QWidget*> m_acTabWidgets;

    QLabel *m_posLabel;
    QLabel *m_srtmLabel;
    QDoubleSpinBox *m_zoomSpinBox;
    QAction *m_displaySrtmAction;
};

#endif // MAINWINDOW_H
