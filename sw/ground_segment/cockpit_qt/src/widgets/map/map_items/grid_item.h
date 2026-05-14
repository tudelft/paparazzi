#pragma once

#include "map_item.h"
#include "widgets/map/obstacle_points/obstacle_grid.h"
#include <QGraphicsRectItem>

class GridItem : public MapItem {
    Q_OBJECT
public:
    GridItem(QString ac_id, float xmin, float ymin, float cell_w, float cell_h, int rows, int cols, int lt, QObject* parent = nullptr);

    void setGridMap(ObstacleGridMap* map) { grid_map = map; }
    void addToMap(MapWidget* map) override;
    void updateGraphics(MapWidget* map, uint32_t update_event) override;
    void updateCell(MapWidget* map, uint32_t update_event, int row, int col);
    void removeFromScene(MapWidget* map) override;

    void setVisible(bool vis);
    void updateRow(int row);

    void setHighlighted(bool h) override;
    void setForbidHighlight(bool fh) override;
    void setEditable(bool ed) override;
    void updateZValue() override;

    void update_origin();

protected:
    QString ac_id;
    Point2DLatLon ltp_origin;

private:
    float xmin, ymin, cell_w, cell_h;
    int rows, cols, lt;
    ObstacleGridMap* grid_map;
    QVector<QVector<QGraphicsRectItem*>> cells;
};