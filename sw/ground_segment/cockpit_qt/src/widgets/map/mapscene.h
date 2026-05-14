#ifndef MAPSCENE_H
#define MAPSCENE_H

#include <QObject>
#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>
#include "item_edit_state_machine.h"

class MapScene : public QGraphicsScene
{
    Q_OBJECT
public:
    explicit MapScene(QObject *parent = nullptr);
    explicit MapScene(qreal x, qreal y, qreal width, qreal height, QObject *parent = nullptr);
    explicit MapScene(const QRectF &sceneRect, QObject *parent = nullptr);

    void setShortcutItems(bool si) {shortcut_items = si;}

protected:
    virtual void mousePressEvent(QGraphicsSceneMouseEvent *mouseEvent);
    virtual void mouseMoveEvent(QGraphicsSceneMouseEvent *mouseEvent);
    virtual void mouseReleaseEvent(QGraphicsSceneMouseEvent *mouseEvent);
    virtual void mouseDoubleClickEvent(QGraphicsSceneMouseEvent *mouseEvent);

signals:
    void eventScene(SmEditEvent, QGraphicsSceneMouseEvent*);

public slots:

private:
    bool shortcut_items;
};

#endif // MAPSCENE_H
