#ifndef MAP2D_H
#define MAP2D_H

#include <QWidget>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QWheelEvent>
#include "tileprovider.h"
#include "tileproviderconfig.h"
#include <QMap>
#include <math.h>
#include <QResizeEvent>
#include "mapscene.h"
#include "maputils.h"

class Map2D : public QGraphicsView
{
    Q_OBJECT
    Q_PROPERTY(QColor color_background MEMBER m_color_background NOTIFY backgroundChanged)
public:
    explicit Map2D(QWidget *parent = nullptr);
    ~Map2D();
    virtual void centerLatLon(Point2DLatLon latLon);
    double zoomBox(Point2DLatLon nw, Point2DLatLon se);
    void toggleTileProvider(QString providerName, bool enable);
    void setMapVisible(bool enable);
    QList<TileProvider*> tileProviders();
    void updateTiles();
    void getViewPoints(Point2DLatLon& nw, Point2DLatLon& sw);
    double zoom() {return _zoom;}
//    int zoomLevel() {return static_cast<int>(ceil(_zoom));}
    virtual void setZoom(double z);
    void zoomCentered(double z, QPoint eventPos);
    void zoomCenteredScene(double z, QPoint center, Point2DPseudoMercator pm);
    void changeZoomTiles(int z);
    int tileSize() {return tile_size;}
    double getRotation();

    void setLayerOpacity(QString providerName, qreal opacity);
    void setLayerZ(QString providerName, int z);
    double scaleFactor() {return pow(2, _zoom - zoomLevel(_zoom));}
    void setMouseLoadTileMask(int mask) {mouse_load_tiles_mask = mask;}

    Point2DLatLon getMouseCoords() { return mouse_coords;}

signals:
    void backgroundChanged(QColor);

public slots:
    void updateSettings();

protected:
    virtual void wheelEvent(QWheelEvent* event) override;
    virtual void mouseMoveEvent(QMouseEvent *event) override;
    virtual void resizeEvent(QResizeEvent *event) override;
    virtual void keyPressEvent(QKeyEvent *event) override;

    Point2DLatLon latlonFromView(QPoint viewPos, int zoom);


    //QGraphicsScene* _scene;
    MapScene* _scene;



private slots:
    void handleTile(TileItem*, TileItem*);


private:
    static QList<TileProviderConfig*> loadConfig(QString filename);

    int mouse_load_tiles_mask;
    double numericZoom;
    double _zoom;
    int tile_size;
    double minZoom;
    double maxZoom;

    int wheelAccumulator;

    QMap<QString, TileProvider*> tile_providers;

    QColor m_color_background;

    Point2DLatLon mouse_coords;
};

#endif // MAP2D_H
