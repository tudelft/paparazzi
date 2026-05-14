#include "mapscene.h"
#include <QDebug>

MapScene::MapScene(QObject *parent) : QGraphicsScene(parent), shortcut_items(false)
{
}


MapScene::MapScene(qreal x, qreal y, qreal width, qreal height, QObject *parent) :
    QGraphicsScene(x, y, width, height, parent), shortcut_items(false)
{
}


MapScene::MapScene(const QRectF &sceneRect, QObject *parent) :
    QGraphicsScene(sceneRect, parent), shortcut_items(false)
{
}


void MapScene::mousePressEvent(QGraphicsSceneMouseEvent *mouseEvent) {
    if(!shortcut_items) {
        QGraphicsScene::mousePressEvent(mouseEvent);
    }
    if(!mouseEvent->isAccepted()) {
        emit eventScene(FPEE_SC_PRESS, mouseEvent);
    }
}

void MapScene::mouseMoveEvent(QGraphicsSceneMouseEvent *mouseEvent) {
    if(!shortcut_items) {
        QGraphicsScene::mouseMoveEvent(mouseEvent);
    }
    if(!mouseEvent->isAccepted()) {
        emit eventScene(FPEE_SC_MOVE, mouseEvent);
    }
}

void MapScene::mouseReleaseEvent(QGraphicsSceneMouseEvent *mouseEvent) {
    if(!shortcut_items) {
        QGraphicsScene::mouseReleaseEvent(mouseEvent);
    }
    if(!mouseEvent->isAccepted()) {
        emit eventScene(FPEE_SC_RELEASE, mouseEvent);
    }
}

void MapScene::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *mouseEvent) {
    if(!shortcut_items) {
        QGraphicsScene::mouseDoubleClickEvent(mouseEvent);
    }
    if(!mouseEvent->isAccepted()) {
        emit eventScene(FPEE_SC_DOUBLE_CLICK, mouseEvent);
    }
}
