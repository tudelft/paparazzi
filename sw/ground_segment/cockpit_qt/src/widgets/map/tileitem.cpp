#include "tileitem.h"
#include <QPainter>
#include <iostream>

TileItem::TileItem(TileItem* mother, int size, Point2DTile coordinates, QGraphicsItem *parent) :
    QGraphicsPixmapItem (parent), SIZE(size),
    inScene(false), _hasData(false), request_status(TILE_NOT_REQUESTED),
    _coordinates(coordinates), _mother(mother)
{
    for(int i=0; i<2; i++) {
        for(int j=0; j<2; j++) {
            _childs[i][j] = nullptr;
        }
    }
}

TileItem::~TileItem() {
    for(int i=0; i<2; i++) {
        for(int j=0; j<2; j++) {
            if(_childs[i][j] != nullptr) {
                delete _childs[i][j];
                _childs[i][j] = nullptr;
            }
        }
    }
}

void TileItem::setPixmap(const QPixmap &pixmap) {
    QGraphicsPixmapItem::setPixmap(pixmap);
    _hasData = true;
}

void TileItem::setAltPixmap(const QPixmap &pixmap) {
    QGraphicsPixmapItem::setPixmap(pixmap);
    _hasData = true;
}


bool TileItem::setInheritedData() {
    //std::cout << "setInheritedData " << coordinates().to_istring().toStdString() << std::endl;
    QPixmap altPixmap(tileSize(), tileSize());
    altPixmap.fill(Qt::transparent);
    bool ancestorsPaint = paintPixmapFromAncestors(&altPixmap);
    bool offspringPaint = paintPixmapFromOffspring(&altPixmap);
    setAltPixmap(altPixmap);

    return ancestorsPaint || offspringPaint;
}

bool TileItem::paintPixmapFromAncestors(QPixmap* altPixmap) {
    if(request_status == TILE_OK) {
        std::cout << "No need to go to ancestors, I already have good data!!!" << std::endl;
        return false;
    }

    TileItem* current = this;

    int posX = 0;
    int posY = 0;

    int dz = 1;

    while(current->mother() != nullptr) {   // && (dz >< _coordinates.zoom())
        posX |= (current->coordinates().xi() & 1) << (dz-1);
        posY |= (current->coordinates().yi() & 1) << (dz-1);

        if(current->mother()->requestStatus() == TILE_OK) {
            int PART_SIZE = tileSize() >> dz;
            QRect rect((posX*tileSize())>>dz, (posY*tileSize())>>dz, PART_SIZE, PART_SIZE);
            QPixmap cropped = current->mother()->pixmap().copy(rect);
            QPixmap scaled = cropped.scaled(tileSize(), tileSize());

            QPainter painter(altPixmap);
            painter.drawPixmap(0, 0, scaled);
            return true;
        }

        current = current->mother();
        dz += 1;
    }
    return false;   // no data in any parent tile
}


bool TileItem::paintPixmapFromOffspring(QPixmap* altPixmap) {
    QPainter painter(altPixmap);
    //std::cout << "seek children" << std::endl;
    for(int i=0; i<2; i++) {
        for(int j=0; j<2; j++) {
            TileItem* c = child(i, j);
            //std::cout << "child found!" << std::endl;
            if(c != nullptr && c->requestStatus() == TILE_OK) {
                QPixmap part = c->pixmap().scaled(tileSize()/2, tileSize()/2);
                painter.drawPixmap(i*tileSize()/2, j*tileSize()/2, part);
            }
        }
    }
    return false;
}


void tileApplyRecursive(TileItem* current, std::function<void(TileItem*)> fn) {
    if(current == nullptr) {
        return;
    }
    fn(current);
    tileApplyRecursive(current->child(0, 0), fn);
    tileApplyRecursive(current->child(1, 0), fn);
    tileApplyRecursive(current->child(0, 1), fn);
    tileApplyRecursive(current->child(1, 1), fn);
}
