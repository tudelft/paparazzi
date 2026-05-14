#ifndef TILEITEM_H
#define TILEITEM_H

#include <QGraphicsPixmapItem>
#include "point2dtile.h"
#include <QPixmap>

enum TileRequestStatus {
    TILE_NOT_REQUESTED,     // tile has not been loaded yet
    TILE_NOT_ON_DISK,       // tile not on disk
    TILE_REQUESTED,         // tile was not on disk, request sent
    TILE_REQUEST_FAILED,    // network request failed
    TILE_OK,                // tile loaded
    TILE_ERROR,             // other generic error
    TILE_DO_NOT_EXISTS,     // tile not in the extent
};

class TileItem : public QGraphicsPixmapItem
{

public:
    TileItem(TileItem* mother, int size, Point2DTile coordinates, QGraphicsItem *parent = nullptr);
    ~TileItem();

    int tileSize() {return SIZE;}
    bool isInScene() { return inScene;}
    bool hasData() {return _hasData;}
    void setInScene(bool in_s) {inScene = in_s;}
    TileRequestStatus requestStatus() {return request_status;}
    void setRequestStatus(TileRequestStatus rs) {request_status = rs;}

    Point2DTile coordinates() {return _coordinates;}
    TileItem* child(int x, int y) {return _childs[x][y];}
    void setChild(TileItem* t, int x, int y) {_childs[x][y] = t;}
    TileItem* mother() {return _mother;}
    bool setInheritedData();
    bool paintPixmapFromAncestors(QPixmap* altPixmap);
    bool paintPixmapFromOffspring(QPixmap* altPixmap);

    virtual void setPixmap(const QPixmap &pixmap);
    void setAltPixmap(const QPixmap &pixmap);


private:
    const int SIZE;
    bool inScene;
    bool _hasData;
    TileRequestStatus request_status;
    const Point2DTile _coordinates;
    TileItem* _childs[2][2];
    TileItem* _mother;
};

void tileApplyRecursive(TileItem* base, std::function<void(TileItem*)> fn);

Q_DECLARE_METATYPE(TileItem*)

#endif // TILEITEM_H
