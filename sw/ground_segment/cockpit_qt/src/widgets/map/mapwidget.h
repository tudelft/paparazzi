#ifndef MAPWIDGET_H
#define MAPWIDGET_H

#include <QGraphicsItem>
#include <QList>
#include <QLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include "map2d.h"
#include "tileprovider.h"
#include "maplayercontrol.h"
#include "imagebutton.h"
#include <QCursor>
#include "configurable.h"
#include "lock_button.h"
#include "papget.h"
#include "wind_sock.h"
#include "../pancontrol.h"

#include "gvf_trajectory.h"
#include "gvf_viewer.h"

#include "grid_viewer.h"
#include "grid_item.h"
#include "circle_item.h"


class ACItemManager;
class ItemEditStateMachine;
class MapItem;
class LayerCombo;
class PathItem;

enum InteractionState {
    PMIS_FLIGHT_PLAN_EDIT,
    PMIS_FROZEN,
    PMIS_OTHER,
};

enum PanState {
    PAN_IDLE,
    PAN_PRESSED,
    PAN_MOVE
};

class MapWidget : public Map2D, public Configurable
{
    Q_OBJECT
    Q_PROPERTY(int ac_arrow_size MEMBER _ac_arrow_size WRITE setAcArrowSize)
    // Q_PROPERTY(bool show_hidden_waypoints WRITE showHiddenWaypoints)
    // Q_PROPERTY(bool show_crash_prediction WRITE showCrashPrediction)
public:
    explicit MapWidget(QWidget *parent = nullptr);

    void addItem(MapItem* map_item);
    void removeItem(MapItem* item);
    virtual void setCursor(const QCursor &);
    void resizeEvent(QResizeEvent *event) override;
    void setPanMask(int mask) {pan_mouse_mask = mask; setMouseLoadTileMask(mask);}
    void itemsForbidHighlight(bool fh);
    void itemsEditable(bool ed);
    void updateHighlights(QString ac_id);
    MapScene* scene() {return _scene;}
    void centerLatLon(Point2DLatLon latLon) override;
    void setZoom(double z) override;

    void configure(QDomElement) override;

    void rotateMap(double rot);
    void setAcArrowSize(int s);
    void showHiddenWaypoints(bool state);
    void showCrashPrediction(bool state);

signals:
    void mouseMoved(QPointF scenePos);
    void itemAdded(MapItem* map_item);
    void itemRemoved(MapItem* item);
    void zoomChanged(double zoom);

protected:
    virtual void mousePressEvent(QMouseEvent *event) override;
    virtual void mouseMoveEvent(QMouseEvent *event) override;
    virtual void mouseReleaseEvent(QMouseEvent *event) override;
    virtual void drawForeground(QPainter *painter, const QRectF &rect) override;
    virtual void wheelEvent(QWheelEvent* event) override;
    virtual void dragEnterEvent(QDragEnterEvent *event) override;
    virtual void dropEvent(QDropEvent *event) override;
    virtual void dragMoveEvent(QDragMoveEvent *event) override;
    virtual bool viewportEvent(QEvent *event) override;

private slots:
    void changeCurrentAC(QString id);
    void updateAircraftItem(pprzlink::Message msg);
    void onWaypointChanged(Waypoint* wp, QString ac_id);
    void onWaypointAdded(Waypoint* wp, QString ac_id);
    void updateTarget(pprzlink::Message msg);
    void updateNavShape(pprzlink::Message msg);
    void onMoveWaypointUi(Waypoint* wp, QString ac_id);
    void onShape(QString sender, pprzlink::Message msg);
    void clearShapes();
    void clearDcShots();
    void onDcShot(QString sender, pprzlink::Message msg);
    void onROTORCRAFT_FP(QString sender, pprzlink::Message msg);
    void onGCSPos(pprzlink::Message msg);
    void onGVF(QString sender, pprzlink::Message msg);
    void onSLAM(QString sender, pprzlink::Message msg);
    void onGridInit(QString sender, pprzlink::Message msg);
    void onGridChanges(QString sender, pprzlink::Message msg);


private:

    enum WidgetContainer {
        WIDGETS_LEFT,
        WIDGETS_RIGHT,
    };

    void handleNewAC(QString ac_id);
    void removeAC(QString ac_id);
    LayerCombo* makeLayerCombo();
    void addWidget(QWidget* w, LockButton* button, WidgetContainer side);
    void setEditorMode();
    void registerWaypoint(WaypointItem* waypoint);
    void updateGraphics(uint32_t update_events);

    QMap<QString, ACItemManager*> ac_items_managers;
    InteractionState interaction_state;
    int drawState;
    ItemEditStateMachine* fp_edit_sm;
    QString current_ac;

    QList<MapItem*> _items;
    QList<Papget*> papgets;

    QMap<uint8_t, MapItem*> shapes;
    QMap<QString, GVF_trajectory*> gvf_trajectories;
    QMap<QString, QVector<int>*> gvf_trajectories_config;
    QList<WaypointItem*> dc_shots;
    MapItem* gcsItem;

    QHBoxLayout* horizontalLayout;

    //ButtonBand
    QVBoxLayout* buttonsLeftLayout;
    QVBoxLayout* columnLeft;
    QVBoxLayout* rightLayout;
    QVBoxLayout* columnRight;
    QVBoxLayout* buttonsRightLayout;
    WindSock* wind_sock;
    QString utc_time_str;
    bool show_utc_time = false;
    PanControl* pan_control;

    QList<LockButton*> buttonsLeft;
    QList<LockButton*> buttonsRight;

    QPoint lastPos;
    PanState pan_state;
    int pan_mouse_mask;

    long shape_bind_id;

    QMap<int, Point2DPseudoMercator> pms;

    int _ac_arrow_size;

    QMenu* mapMenu;
    QMenu* menu_clear_track;
    QAction* show_hidden_wp_action;
    QAction* show_crash_prediction_action;

    // Obstacles
    GridItem* grid_item = nullptr;
    ObstacleGridMap* obstacle_grid_map = nullptr;
    QImage slam_grid_image;
    bool slam_grid_visible = true;
    bool obstacles_visible = false;
    bool show_utm_grid = false;
    QAction* utm_grid_action;
    QAction* utc_time_action;
    QList<QPair<CircleItem*, QDateTime>> slam_obstacles;
};

#endif // MAPWIDGET_H
