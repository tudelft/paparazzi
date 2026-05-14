#ifndef UI_DISPATCHER_H
#define UI_DISPATCHER_H

#include <QObject>
#include "waypoint.h"
#include "point2dlatlon.h"
#include "PprzToolbox.h"
#include "PprzApplication.h"

class Setting;

class DispatcherUi : public PprzTool
{
    Q_OBJECT

public:
    explicit DispatcherUi(PprzApplication* app, PprzToolbox* toolbox);
    static DispatcherUi* get() {
        return pprzApp()->toolbox()->dispatcherUi();
    }

    QString getSelectedAcId() { return selected_ac_id;}

signals:
    void ac_selected(QString);
    void ac_deleted(QString);
    void new_ac_config(QString);
    void move_waypoint_ui(Waypoint*, QString ac_id);   //wp moved from UI
    void settingUpdated(QString ac_id, Setting*, float value);
    void mapRotated(double);    // map view rotation in degrees
    void centerMap(Point2DLatLon pos);
    void fitToWindow();
    void toggleAcLabel(QString ac_id, bool visible);
    void clearTrackForAc(QString ac_id);
    void resizeTrackForAc(QString ac_id, int size);
    void getTrackSizeForAc(QString ac_id, int &size);
    void loadKmlShape(QString file);
    void loadSectorShape(QString file);
    //void create_waypoint(Waypoint*);

    // GVF viewer signals
    void gvf_settingUpdated(QString ac_id, QVector<int> *gvfViewer_config);
    void gvf_defaultFieldSettings(QString ac_id, int area, int xpts, int ypts);
    void gvf_zlimits(QString ac_id, float minz, float maxz);
    void slamGridVisibilityChanged(bool visible);
    void obstacleVisibilityChanged(bool visible);

public slots:

private:
    QString selected_ac_id;

};

#endif // UI_DISPATCHER_H
