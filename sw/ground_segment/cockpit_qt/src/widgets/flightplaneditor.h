#ifndef FLIGHTPLANEDITOR_H
#define FLIGHTPLANEDITOR_H

#include <QtWidgets>
#include <libxml/parser.h>
#include "waypoint.h"

namespace Ui {
class FlightPlanEditor;
}

struct AttributeDef {
    const xmlChar* name;
    bool required;
};

class FlightPlanEditor : public QWidget
{
    Q_OBJECT
public:
    explicit FlightPlanEditor(QString ac_id, QWidget *parent = nullptr);
    ~FlightPlanEditor();

    int parse(QString filename);

    bool validate();
    QByteArray output();

signals:
    void waypointEdited(Waypoint*);

private:
    Ui::FlightPlanEditor *ui;

    static void applyRecursive(QTreeWidgetItem* item, std::function<void(QTreeWidgetItem*)> f);

    void onItemClicked(QTreeWidgetItem *item, int column);
    void onItemDoubleClicked(QTreeWidgetItem *item, int column);
    void onAttributeChanged(QTreeWidgetItem *item, int column);
    void onNavStatus();

    void onMoveWaypointUi(Waypoint*, QString ac_id);
    void onWaypointAdded(Waypoint*, QString ac_id);

    QTreeWidgetItem* populate(xmlNodePtr node, QTreeWidgetItem* parent=nullptr, QTreeWidgetItem* prev=nullptr);

    void openElementsContextMenu(QPoint pos);
    void openAttributesContextMenu(QPoint pos);
    void onArrowClicked(bool up);

    QString sumaryFromNode(xmlNodePtr node);

    QList<struct AttributeDef> attributeDefs(const xmlChar* nodeName);
    QStringList elementDefs(xmlElementContentPtr c);

    QString ac_id;

    xmlDocPtr doc;
    xmlDtdPtr dtd;

    QMap<QTreeWidgetItem*, xmlNodePtr> nodes;
    QMap<QTreeWidgetItem*, xmlNodePtr> attributes_nodes;
    QMap<QTreeWidgetItem*, Waypoint*> waypoints;
    QMap<uint8_t, QTreeWidgetItem*> blocks;

    uint8_t last_block;

    QTreeWidgetItem* waypoints_item;

    bool readOnly;

};

#endif // FLIGHTPLANEDITOR_H
