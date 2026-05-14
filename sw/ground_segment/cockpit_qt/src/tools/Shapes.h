#ifndef SHAPES_H
#define SHAPES_H

#include <QObject>
#include <QMap>
#include "pprz_dispatcher.h"
#include "shape_item.h"

class Shapes : public QObject
{
    Q_OBJECT
public:
    static Shapes* get();
    void init();

private:
    explicit Shapes(QObject *parent = nullptr);
    static Shapes* singleton;

    void handleShapeMessage(QString sender, pprzlink::Message msg);
    
    QMap<int, ShapeItem*> _shapes;
};

#endif // SHAPES_H
