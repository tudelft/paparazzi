/*
**************** Editing ONE (single) flight plan **************************/

/*
**************** Editing ONE (single) flight plan **************************/

/***************** Editing ONE (single) flight plan **************************/

#include "flightplaneditor.h"
#include "AircraftManager.h"
#include "gcs_utils.h"
#include <libxml/tree.h>
#include "dispatcher_ui.h"
#include "ui_flightplaneditor.h"

QString getProp(xmlNodePtr node, const QString& tag) {
    auto att = (char*)xmlGetProp((node), BAD_CAST (tag.toUtf8().constData()));
    if(att) {
        return QString(att);
    } else {
        return QString();
    }
}

// function to respect indentation
void xml_add_next_sibling(xmlNodePtr node, xmlNodePtr new_node);
void xml_add_child(xmlNodePtr parent, xmlNodePtr new_node);

FlightPlanEditor::FlightPlanEditor(QString ac_id, QWidget *parent) : QWidget(parent),
    ui(new Ui::FlightPlanEditor),
    ac_id(ac_id), doc(nullptr), dtd(nullptr), last_block(0), waypoints_item(nullptr), readOnly(false)
{
    ui->setupUi(this);
    if (layout()) {
        layout()->setContentsMargins(5, 5, 5, 5);
    }
    
    // Reduce internal tree widgets so the QTabWidget (and main window) can shrink vertically 
    ui->tree->setMinimumHeight(40);
    ui->attributes->setMinimumHeight(40);

    ui->error_label->hide();
    ui->save_button->setIcon(style()->standardIcon(QStyle::SP_DialogSaveButton));
    ui->up_button->setIcon(style()->standardIcon(QStyle::SP_ArrowUp));
    ui->down_button->setIcon(style()->standardIcon(QStyle::SP_ArrowDown));

    if(AircraftManager::get()->getAircraft(ac_id)->isReal()) {
        readOnly = true;
    }

    auto config = AircraftManager::get()->getAircraft(ac_id)->getConfig();
    int result = parse(config->getFlightPlanUri());

    if(result != 0) {
        ui->tree->hide();
        ui->validate_button->hide();
        ui->save_button->hide();
        ui->attributes->hide();
        ui->up_button->hide();
        ui->down_button->hide();
        ui->error_label->show();
    }

    if(!readOnly) {
        connect(ui->validate_button, &QPushButton::clicked, this, [=]() {
            bool result = validate();
            if(result) {
                QMessageBox::information(this, "DTD Validation", "XML is valid!");
            } else {
                QMessageBox::critical(this, "DTD Validation", "XML not valid! Try check stderr for more information.");
            }
        });

        connect(ui->save_button, &QPushButton::clicked, this, [=]() {
            auto data = output();
            auto out_path = QUrl(config->getFlightPlanUri()).toLocalFile();
            out_path = QFileDialog::getSaveFileName(this, "save btu", out_path);

            if(out_path != "") {
                QFile out(out_path);
                if(out.open(QIODevice::WriteOnly | QIODevice::Text)) {
                    QTextStream stream( &out );
                    stream << data;
                    out.close();
                }
            }
        });

        connect(ui->up_button, &QPushButton::clicked, this, [=]() {
            onArrowClicked(true);
        });

        connect(ui->down_button, &QPushButton::clicked, this, [=]() {
            onArrowClicked(false);
        });

        connect(ui->attributes, &QTreeWidget::itemDoubleClicked, this, [=](QTreeWidgetItem * item, int column) {
            if(column == 1) {
                ui->attributes->editItem(item, column);
            }
        });

        connect(ui->tree, &QTreeWidget::itemClicked, this, &FlightPlanEditor::onItemClicked);
        connect(ui->tree, &QTreeWidget::customContextMenuRequested, this, &FlightPlanEditor::openElementsContextMenu);
        connect(AircraftManager::get(), &AircraftManager::waypoint_added, this, &FlightPlanEditor::onWaypointAdded);
        connect(ui->attributes, &QTreeWidget::customContextMenuRequested, this, &FlightPlanEditor::openAttributesContextMenu);

    } else {
        ui->validate_button->hide();
        ui->save_button->hide();
        ui->up_button->hide();
        ui->down_button->hide();

        connect(ui->tree, &QTreeWidget::itemDoubleClicked, this, [=](QTreeWidgetItem * item, int column) {
            qDebug() << column << item->text(0) << item->text(1);
        });

        connect(ui->tree, &QTreeWidget::itemClicked, this, &FlightPlanEditor::onItemClicked);
        connect(ui->tree, &QTreeWidget::itemDoubleClicked, this, &FlightPlanEditor::onItemDoubleClicked);
        connect(AircraftManager::get()->getAircraft(ac_id)->getStatus(),
                &AircraftStatus::nav_status, this, &FlightPlanEditor::onNavStatus);

    }

    connect(AircraftManager::get(), &AircraftManager::waypoint_changed, this, &FlightPlanEditor::onMoveWaypointUi);

    if(doc) {
        xmlNode *root_element = xmlDocGetRootElement(doc);
        if(QString((char*)root_element->name) == "dump") {
            auto child = xmlFirstElementChild(root_element);
            while(child) {
                if(QString((char*)child->name) == "flight_plan") {
                    root_element = child;
                    break;
                }
                child = xmlNextElementSibling(child);
            }
        }
        populate(root_element);
    }
}

FlightPlanEditor::~FlightPlanEditor() {
    xmlFreeDoc(doc);
    delete ui;
}

int FlightPlanEditor::parse(QString filename) {
    LIBXML_TEST_VERSION

    // parse the file
    doc = xmlReadFile(filename.toStdString().c_str(), nullptr, XML_PARSE_NOWARNING);
    /* check if parsing succeeded */
    if (doc == nullptr) {
        ui->error_label->setText("Failed to parse " + filename);
        qDebug() << "Failed to parse " + filename;
        return -1;
    }

    auto internal_dtd = xmlGetIntSubset(doc);
    if(internal_dtd != nullptr) {
        auto systemId = (char*)internal_dtd->SystemID;
        QUrl url(filename);
        auto finfo = QFileInfo(url.toLocalFile());
        auto dir = finfo.absoluteDir();
        auto dtd_path = finfo.absoluteDir().absoluteFilePath(systemId);
        dtd = xmlParseDTD(NULL, BAD_CAST dtd_path.toStdString().c_str());
    }

    if(internal_dtd == nullptr || dtd == nullptr) {
        auto dtd_path = appConfig()->value("PAPARAZZI_HOME").toString() + "/conf/flight_plans/flight_plan.dtd";
        dtd = xmlParseDTD(NULL, BAD_CAST dtd_path.toStdString().c_str());
    }

    if(dtd == nullptr) {
        ui->error_label->setText("Failed to parse DTD");
        qDebug() << "Failed to parse DTD";
        return -1;
    }
return 0;
}

QTreeWidgetItem* FlightPlanEditor::populate(xmlNodePtr cur_node, QTreeWidgetItem* parent, QTreeWidgetItem* prev) {
    QTreeWidgetItem* item = nullptr;
    if (cur_node->type == XML_ELEMENT_NODE) {
        QString name = (const char*)cur_node->name;
        if(parent != nullptr) {
            if(prev == nullptr) {
                item = new QTreeWidgetItem(parent);
            } else {
                item = new QTreeWidgetItem(parent, prev);
            }
        } else {
            if(prev == nullptr) {
                item = new QTreeWidgetItem(ui->tree);
            } else {
                item = new QTreeWidgetItem(ui->tree, prev);
            }
        }
        item->setText(0, name);
        //item->setFlags(item->flags() | Qt::ItemIsEditable);
        nodes[item] = cur_node;

        item->setText(1, sumaryFromNode(cur_node));

        if(name == "waypoints") {
            waypoints_item = item;
        } else if(name == "waypoint") {
            QString wp_name = (char*)xmlGetProp(cur_node, BAD_CAST "name");
            auto fp = AircraftManager::get()->getAircraft(ac_id)->getFlightPlan();
            try {
                auto wp = fp->getWaypoint(wp_name);
                waypoints[item] = wp;
            }  catch (std::runtime_error &e) {
                qDebug() << "Waypoint " + wp_name + " not found!";
            }

        } else if(name == "block") {
            auto cno = xmlGetProp(cur_node, BAD_CAST "no");
            if(cno) {
               QString no = (char*)cno;
               blocks[no.toUInt()] = item;
            }
        }

        QTreeWidgetItem* prev_item = nullptr;
        for (xmlNodePtr child = cur_node->children; child; child = child->next) {
            prev_item = populate(child, item, prev_item);
        }

        if(parent == nullptr && prev == nullptr) {
            item->setExpanded(true);
        }
    } else {
        auto elt = xmlGetDtdElementDesc(dtd, cur_node->parent->name);
        if(elt) {
            xmlElementContentPtr c=elt->content;
            auto elts = elementDefs(c);
            if(elts.contains("#PCDATA")) {
                //QString content = (char*)node->children->content;
                item = new QTreeWidgetItem(parent);
                item->setText(0, "PCDATA");
                //item->setFlags(item->flags() | Qt::ItemIsEditable);
                nodes[item] = cur_node;

            }
        }
    }
    return item;
}

QString FlightPlanEditor::sumaryFromNode(xmlNodePtr node) {
    QString node_name = (const char*)node->name;

    if(node_name == "block" || node_name == "flight_plan" || node_name == "waypoint") {
        return getProp(node, "name");
    } else if(node_name == "circle") {
        return "wp=" + getProp(node, "wp") + ", radius=" + getProp(node, "radius");
    } else if(node_name == "sector") {
        auto name = getProp(node, "name");
        auto color = getProp(node, "color");
        if (color != "") {
            return name + QString(", %1").arg(color);
        } else {
            return name;
        }
    } else if(node_name == "corner") {
        return getProp(node, "name");
    } else if(node_name == "variable") {
        auto var = getProp(node, "var");
        return var;
    } else if(node_name == "module") {
        return getProp(node, "name");
    } else if(node_name == "define") {
        return getProp(node, "name");
    } else if(node_name == "configure") {
        return getProp(node, "name");
    } else if(node_name == "exception") {
        auto cond = getProp(node, "cond");
        auto deroute = getProp(node, "deroute");
        return QString("%1 -> %2").arg(cond, deroute);
    } else if(node_name == "while") {
        return getProp(node, "cond");
    } else if(node_name == "for") {
        auto var = getProp(node, "var");
        auto from = getProp(node, "from");
        auto to = getProp(node, "to");
        return QString("%1: %2 -> %3").arg(var, from, to);
    } else if(node_name == "heading") {
        return getProp(node, "course");
    } else if(node_name == "attitude") {
        return getProp(node, "roll");
    } else if(node_name == "go") {
        return getProp(node, "wp");
    } else if(node_name == "set") {
        auto var = getProp(node, "var");
        auto value = getProp(node, "value");
        return QString("%1 := %2").arg(var, value);
    } else if(node_name == "call") {
        return getProp(node, "fun");
    } else if(node_name == "call_once") {
        return getProp(node, "fun");
    } else if(node_name == "eight") {
        return getProp(node, "center");
    } else if(node_name == "oval") {
        return getProp(node, "p1") + "-" + getProp(node, "p2");
    } else if(node_name == "deroute") {
        return getProp(node, "block");
    } else if(node_name == "stay") {
        return getProp(node, "wp");
    }

    return QString();
}

void FlightPlanEditor::onArrowClicked(bool up) {
    auto selected = ui->tree->selectedItems();
    if(selected.length() != 1) {
        return;
    }
    auto item = selected[0];
    auto parent = item->parent();
    if(item->parent() == nullptr) {
        return;
    }

    int index = parent->indexOfChild(item);
    // check if it can be moved in this direction
    if((up && index == 0) ||
       (!up && index == parent->childCount()-1)) {
        return;
    }

    auto node = nodes[item];

    if(up) {
        auto prev = node->prev;
        while(prev->type != XML_ELEMENT_NODE) {
            if(prev == nullptr) {
                return;
            }
            prev = prev->prev;
        }
        xmlUnlinkNode(node);
        xmlAddPrevSibling(prev, node);
        parent->takeChild(index);
        parent->insertChild(index-1, item);
    }
    else {
        auto next = node->next;
        while(next->type != XML_ELEMENT_NODE) {
            if(next == nullptr) {
                return;
            }
            next = next->next;
        }
        xmlUnlinkNode(node);
        xmlAddNextSibling(next, node);
        parent->takeChild(index);
        parent->insertChild(index+1, item);
    }

    ui->tree->clearSelection();
    item->setSelected(true);

}

void FlightPlanEditor::openElementsContextMenu(QPoint pos) {

    auto selected = ui->tree->selectedItems();
    if(selected.length() != 1) {
        return;
    }

    auto item = selected[0];

    auto node = nodes[item];
    if(node->type != XML_ELEMENT_NODE) {
        return;
    }
    auto parent = node->parent;

    QMenu *menu = new QMenu(this);

    /// Delete action
    auto delete_action = menu->addAction("Delete");
    connect(delete_action, &QAction::triggered, this, [=]() {
        xmlUnlinkNode(node);
        xmlFreeNode(node);

        std::function<QList<QTreeWidgetItem*>(QTreeWidgetItem*)> getAllChildren = [&getAllChildren](QTreeWidgetItem* it){
            QList<QTreeWidgetItem*> list;
            for(int i=0; i< it->childCount(); i++) {
                list.append(getAllChildren(it->child(i)));
            }
            list.append(it);
            return list;
        };

        for(auto it: getAllChildren(item)) {
            if(it == waypoints_item) {
                waypoints_item = nullptr;
            }
            nodes.remove(it);
            delete it;
        }


    });

    /// Add Child menu
    auto childMenu = new QMenu(menu);
    childMenu->setTitle("Add child");
    auto elt = xmlGetDtdElementDesc(dtd, node->name);
    if(elt != nullptr) {
        xmlElementContentPtr c=elt->content;
        auto elements = elementDefs(c);
        elements.removeAll("#PCDATA");
        for(auto &name: elements) {
            auto addchild_action = childMenu->addAction(name);
            connect(addchild_action, &QAction::triggered, this, [=]() {
                item->setExpanded(true);
                xmlNodePtr new_node = xmlNewNode(nullptr, BAD_CAST name.toStdString().c_str());
                for(auto att: attributeDefs(new_node->name)) {
                    if(att.required) {
                        xmlSetProp(new_node, att.name, BAD_CAST "???");
                    }
                }
                xml_add_child(node, new_node);

                if(QString((char*)new_node->name) == "waypoint") {
                    xmlSetProp(new_node, BAD_CAST "x", BAD_CAST "12");
                    xmlSetProp(new_node, BAD_CAST "y", BAD_CAST "12");
                    auto fp = AircraftManager::get()->getAircraft(ac_id)->getFlightPlan();
                    auto wp = fp->addWaypoint("???", 12, 12);
                    xmlSetProp(new_node, BAD_CAST "name", BAD_CAST wp->getName().toUtf8().constData());
                    populate(new_node, item);
                    emit AircraftManager::get()->waypoint_added(wp, ac_id);
                } else {
                    populate(new_node, item);
                }
            });
        }
        if(childMenu->actions().count() > 0) {
            menu->addMenu(childMenu);
        }
    }


    ///Copy after action
    auto copy_action = menu->addAction("Copy after");
    connect(copy_action, &QAction::triggered, this, [=]() {
        auto copy = xmlCopyNode(node, 1);
        xml_add_next_sibling(node, copy);
        if(QString((char*)copy->name) == "waypoint") {
            //If this is a waypoint, change its name and create the waypoint
            xmlSetProp(copy, BAD_CAST "name", BAD_CAST "???");
            auto fp = AircraftManager::get()->getAircraft(ac_id)->getFlightPlan();
            auto wp = waypoints[item];
            double x, y;
            wp->getRelative(fp->getFrame(), x, y);
            auto wp_copy = fp->addWaypoint("???", x, y);
            xmlSetProp(copy, BAD_CAST "name", BAD_CAST wp_copy->getName().toUtf8().constData());
            //populate must be called before emitting the signal
            populate(copy, item->parent(), item);
            emit AircraftManager::get()->waypoint_added(wp_copy, ac_id);
        } else {
            populate(copy, item->parent(), item);
        }




    });

    /// Add after menu
    if(parent != nullptr && parent->name != nullptr) {
        auto afterMenu = new QMenu(menu);
        afterMenu->setTitle("Add after");
        elt = xmlGetDtdElementDesc(dtd, parent->name);
        if(elt != nullptr) {
            xmlElementContentPtr c=elt->content;
            auto elements = elementDefs(c);
            elements.removeAll("#PCDATA");
            for(auto &name: elements) {
                auto addafter_action = afterMenu->addAction(name);
                connect(addafter_action, &QAction::triggered, this, [=]() {
                    xmlNodePtr new_node = xmlNewNode(nullptr, BAD_CAST name.toStdString().c_str());
                    for(auto att: attributeDefs(new_node->name)) {
                        if(att.required) {
                            xmlSetProp(new_node, att.name, BAD_CAST "???");
                        }
                    }

                    xml_add_next_sibling(node, new_node);

                    if(QString((char*)new_node->name) == "waypoint") {
                        //If this is a waypoint, change its name and create the waypoint
                        auto fp = AircraftManager::get()->getAircraft(ac_id)->getFlightPlan();
                        auto wp = waypoints[item];
                        double x, y;
                        wp->getRelative(fp->getFrame(), x, y);
                        xmlSetProp(new_node, BAD_CAST "x", BAD_CAST QString::number(x).toStdString().c_str());
                        xmlSetProp(new_node, BAD_CAST "y", BAD_CAST QString::number(y).toStdString().c_str());
                        auto wp_copy = fp->addWaypoint("???", x, y);
                        xmlSetProp(new_node, BAD_CAST "name", BAD_CAST wp_copy->getName().toStdString().c_str());
                        //populate must be called before emitting the signal
                        populate(new_node, item->parent(), item);
                        emit AircraftManager::get()->waypoint_added(wp_copy, ac_id);
                    } else {
                        populate(new_node, item->parent(), item);
                    }

                });
            }
            if(afterMenu->actions().count() > 0) {
                menu->addMenu(afterMenu);
            }
        }
    }

    menu->popup(ui->tree->viewport()->mapToGlobal(pos));

}


void FlightPlanEditor::openAttributesContextMenu(QPoint pos) {
    auto selected = ui->tree->selectedItems();
    if(selected.length() != 1) {
        return;
    }
    auto node_item = selected[0];
    auto node = nodes[node_item];



    auto elt = xmlGetDtdElementDesc(dtd, node->name);
    (void)elt;
    if(elt != nullptr) {
        QMenu *menu = new QMenu(this);

        auto dtd_attributes = attributeDefs(node->name);

        auto att_selected = ui->attributes->selectedItems();
        if(att_selected.length() == 1) {
            auto selected_name = att_selected[0]->text(0);
            for(auto att: dtd_attributes) {
                if(att_selected[0]->text(0) == QString((char*)att.name) && !att.required) {
                    /// Delete action
                    auto delete_action = menu->addAction("Delete");
                    connect(delete_action, &QAction::triggered, this, [=]() {
                        xmlUnsetProp(node, att.name);
                        onItemClicked(node_item, 1);
                    });
                }
            }
        }

        // add attributes not currently present
        auto addAttMenu = new QMenu("Add attribute", menu);
        for(auto att: dtd_attributes) {
            qDebug() << QString((char*)att.name);
            auto prop = xmlGetProp(node, att.name);
            if(prop == nullptr) {
                // Add attribute if not already there
                auto add_attribute_action = addAttMenu->addAction(QString((char*)att.name));
                connect(add_attribute_action, &QAction::triggered, this, [=]() {
                    xmlSetProp(node, att.name, BAD_CAST "???");
                    onItemClicked(node_item, 1);
                });
            }
        }
        if(addAttMenu->actions().count() > 0) {
            menu->addMenu(addAttMenu);
        }

        if(QString((char*)node->name) == "waypoint") {
            if(xmlGetProp(node, BAD_CAST "x")) {
                auto convert_action = menu->addAction("Convert to lat/lon");
                connect(convert_action, &QAction::triggered, this, [=]() {
                    xmlUnsetProp(node, BAD_CAST "x");
                    xmlUnsetProp(node, BAD_CAST "y");
                    auto wp = waypoints[node_item];
                    auto lat = wp->getLat();
                    auto lon = wp->getLon();
                    xmlSetProp(node, BAD_CAST "lat", BAD_CAST QString::number(lat, 'f', 7).toStdString().c_str());
                    xmlSetProp(node, BAD_CAST "lon", BAD_CAST QString::number(lon, 'f', 7).toStdString().c_str());
                    onItemClicked(node_item, 1);
                });
            } else if(xmlGetProp(node, BAD_CAST "lat")) {
                auto convert_action = menu->addAction("Convert to x/y");
                connect(convert_action, &QAction::triggered, this, [=]() {
                    xmlUnsetProp(node, BAD_CAST "lat");
                    xmlUnsetProp(node, BAD_CAST "lon");
                    auto wp = waypoints[node_item];
                    double x, y;
                    auto fp = AircraftManager::get()->getAircraft(ac_id)->getFlightPlan();
                    wp->getRelative(fp->getFrame(), x, y);
                    xmlSetProp(node, BAD_CAST "x", BAD_CAST QString::number(x).toStdString().c_str());
                    xmlSetProp(node, BAD_CAST "y", BAD_CAST QString::number(y).toStdString().c_str());
                    onItemClicked(node_item, 1);
                });
            }
        }


        menu->popup(ui->attributes->viewport()->mapToGlobal(pos));
    }
}

void FlightPlanEditor::onAttributeChanged(QTreeWidgetItem *item, int column) {
    if(column != 1) {
        return;
    }
    auto node = attributes_nodes[item];

    if(node->type == XML_TEXT_NODE) {
        // for the PCDATA node of header
        xmlNodeSetContent(node, BAD_CAST item->text(1).toUtf8().constData());
        return;
    }

    xmlSetProp(node, BAD_CAST item->text(0).toUtf8().constData(), BAD_CAST item->text(1).toUtf8().constData());

    auto selected = ui->tree->selectedItems();
    if(selected.length() == 0) {
        return;
    }

    auto node_item = selected[0];
    auto att_name = item->text(0);

    if(node_item->text(0) != "waypoint"){
        return;
    }

    auto wp = waypoints[node_item];

    if(att_name == "x" || att_name == "y" || att_name == "lat" || att_name == "lon") {
        try {
            auto new_value = parse_coordinate(item->text(1));
            Waypoint::WpFrame frame = AircraftManager::get()->getAircraft(ac_id)->getFlightPlan()->getFrame();
            if(att_name == "x") {
                double x, y;
                wp->getRelative(frame, x, y);
                wp->setRelative(frame, new_value, y);
            } else if(att_name == "y") {
                double x, y;
                wp->getRelative(frame, x, y);
                wp->setRelative(frame, x, new_value);
            } else if(att_name == "lat") {
                wp->setLat(new_value);
            } else if(att_name == "lon") {
                wp->setLon(new_value);
            }
            emit AircraftManager::get()->waypoint_changed(wp, ac_id);
        }  catch (std::runtime_error &e) {
            qDebug() << "not a coordinate";
            return;
        }
    } else if(att_name == "name") {
        auto requested_name = item->text(1);
        auto fp = AircraftManager::get()->getAircraft(ac_id)->getFlightPlan();
        auto new_name = fp->requestNewName(requested_name);
        wp->setName(new_name);
        if(requested_name != new_name) {
            xmlSetProp(node, BAD_CAST "name", BAD_CAST new_name.toUtf8().constData());
        }
        node_item->setText(1, sumaryFromNode(node));
        emit AircraftManager::get()->waypoint_changed(wp, ac_id);
    }


}

void FlightPlanEditor::onMoveWaypointUi(Waypoint* wp, QString acid) {
    if(acid != ac_id) {
        return;
    }
    for(auto it=waypoints.begin(); it!=waypoints.end(); ++it) {
        if(it.value() == wp) {
            auto node = nodes[it.key()];

            if(xmlGetProp(node, BAD_CAST "x") || xmlGetProp(node, BAD_CAST "y")) {
                auto frame = AircraftManager::get()->getAircraft(ac_id)->getFlightPlan()->getFrame();
                double x, y;
                wp->getRelative(frame, x, y);
                xmlSetProp(node, BAD_CAST "x", BAD_CAST QString::number(x).toStdString().c_str());
                xmlSetProp(node, BAD_CAST "y", BAD_CAST QString::number(y).toStdString().c_str());
            } else if(xmlGetProp(node, BAD_CAST "lat")  || xmlGetProp(node, BAD_CAST "lon")) {
                auto lat = wp->getLat();
                auto lon = wp->getLon();
                xmlSetProp(node, BAD_CAST "lat", BAD_CAST QString::number(lat, 'f', 7).toStdString().c_str());
                xmlSetProp(node, BAD_CAST "lon", BAD_CAST QString::number(lon, 'f', 7).toStdString().c_str());
            }
            if(!readOnly) {
                ui->tree->clearSelection();
                it.key()->setSelected(true);
            }
            if(ui->tree->selectedItems().size() > 0 && ui->tree->selectedItems().first() == it.key()) {
                onItemClicked(it.key(), 1);
            }
        }
    }
}

void FlightPlanEditor::onWaypointAdded(Waypoint* wp, QString ac_id) {
    if(!waypoints.values().contains(wp)) {
        if(!waypoints_item) {
            auto fp_item = ui->tree->topLevelItem(0);
            if(nodes.contains(fp_item)) {
                auto waypoints_node = xmlNewNode(nullptr, BAD_CAST "waypoints");
                xmlAddChild(nodes[fp_item], waypoints_node);
                populate(waypoints_node, fp_item);
            }
        }
        if(waypoints_item) {
            auto waypoints_node = nodes[waypoints_item];
            xmlNodePtr new_node = xmlNewNode(nullptr, BAD_CAST "waypoint");
            xmlSetProp(new_node, BAD_CAST "name", BAD_CAST wp->getName().toUtf8().constData());

            double x, y;
            auto fp = AircraftManager::get()->getAircraft(ac_id)->getFlightPlan();
            wp->getRelative(fp->getFrame(), x, y);
            xmlSetProp(new_node, BAD_CAST "x", BAD_CAST QString::number(x).toStdString().c_str());
            xmlSetProp(new_node, BAD_CAST "y", BAD_CAST QString::number(y).toStdString().c_str());

            xml_add_child(waypoints_node, new_node);
            populate(new_node, waypoints_item);
        }

    }
}

void FlightPlanEditor::applyRecursive(QTreeWidgetItem* item, std::function<void(QTreeWidgetItem*)> f) {
    for(int c=0; c<item->childCount(); ++c) {
        applyRecursive(item->child(c), f);
    }
    f(item);
}

void FlightPlanEditor::onItemClicked(QTreeWidgetItem *item, int column) {
    (void)column;

    disconnect(ui->attributes, &QTreeWidget::itemChanged, this, &FlightPlanEditor::onAttributeChanged);

    ui->attributes->clear();
    attributes_nodes.clear();
    auto node = nodes[item];
    for(xmlAttrPtr att=node->properties; att!=nullptr; att=att->next) {
        QString att_name = (char*)att->name;
        QString att_val = (char*)att->children->content;

        auto item = new QTreeWidgetItem(ui->attributes);
        item->setText(0, att_name);
        item->setText(1, att_val);
        item->setFlags(item->flags() | Qt::ItemIsEditable);
        attributes_nodes[item] = node;
    }

    if(QString((char*)node->name) == "text") {
        QString content = (char*)node->content;
        auto item = new QTreeWidgetItem(ui->attributes);
        item->setText(0, "PCDATA");
        item->setText(1, content);
        item->setFlags(item->flags() | Qt::ItemIsEditable);
        attributes_nodes[item] = node;
    }

    connect(ui->attributes, &QTreeWidget::itemChanged, this, &FlightPlanEditor::onAttributeChanged);

}

void FlightPlanEditor::onItemDoubleClicked(QTreeWidgetItem *item, int column) {
    (void)column;
    auto node = nodes[item];
    if(item->text(0)=="block") {
       auto cno = xmlGetProp(node, BAD_CAST "no");
       if(cno) {
           QString no = (char*)cno;
           pprzlink::Message msg(PprzDispatcher::get()->getDict()->getDefinition("JUMP_TO_BLOCK"));
           msg.addField("ac_id", ac_id);
           msg.addField("block_id", no.toUInt());
           PprzDispatcher::get()->sendMessage(msg);
       }
    }

}

void FlightPlanEditor::onNavStatus() {
    auto last = blocks[last_block];
    if(last == nullptr) {
        return;
    }

    applyRecursive(last, [=](QTreeWidgetItem* item){
        for(int i=0; i<item->columnCount(); ++i) {
            item->setBackground(i, Qt::transparent);
        }
    });

    auto msg = AircraftManager::get()->getAircraft(ac_id)->getStatus()->getMessage("NAV_STATUS");
    if(msg) {
        uint8_t cur_block, cur_stage;
        //uint32_t block_time, stage_time;
        //float target_lat, target_long, target_climb, target_alt, target_course, dist_to_wp;
        msg->getField("cur_block", cur_block);
        msg->getField("cur_stage", cur_stage);

        applyRecursive(blocks[cur_block], [=](QTreeWidgetItem* item){
            QColor color = Qt::darkGreen;
            auto node = nodes[item];
            auto cno = xmlGetProp(node, BAD_CAST "no");
            if(item->text(0) != "block" && cno) {
                QString no = (char*)cno;
                if(no.toUInt() == cur_stage) {
                    color = Qt::green;
                }
            }

            for(int i=0; i<item->columnCount(); ++i) {
                item->setBackground(i, color);
            }
        });

        last_block = cur_block;

    }
}

QStringList FlightPlanEditor::elementDefs(xmlElementContentPtr c) {
    QStringList list;
    if(c == NULL) {
        return list;
    }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    switch(c->type) {
    case XML_ELEMENT_CONTENT_PCDATA:
        list.append("#PCDATA");
        break;
    case XML_ELEMENT_CONTENT_ELEMENT:
        list.append((char*)c->name);
        break;
    case XML_ELEMENT_CONTENT_SEQ:
    case XML_ELEMENT_CONTENT_OR:
        list.append(elementDefs(c->c1));
        list.append(elementDefs(c->c2));
        break;
    }
#pragma GCC diagnostic pop

    list.removeDuplicates();
    return list;
}

QList<struct AttributeDef> FlightPlanEditor::attributeDefs(const xmlChar* nodeName) {
    QList<struct AttributeDef> list;
    auto elt = xmlGetDtdElementDesc(dtd, nodeName);

    for(xmlAttributePtr att = elt->attributes; att!= NULL; att=att->nexth) {
        struct AttributeDef def = {
            att->name,
            false
        };

        if(att->def == XML_ATTRIBUTE_REQUIRED) {
            def.required = true;
        }
        list.append(def);
    }
    return list;
}

QByteArray FlightPlanEditor::output() {
    xmlChar *xmlbuff;
    int buffersize;

    xmlDocDumpFormatMemory(doc, &xmlbuff, &buffersize, 1);

    QByteArray data((char*)xmlbuff);
    /*
     * Free associated memory.
     */
    xmlFree(xmlbuff);
    return data;
}

bool FlightPlanEditor::validate() {
    auto valid_ctcx = xmlNewValidCtxt();
    int valid = xmlValidateDtd(valid_ctcx, doc, dtd);
    xmlFreeValidCtxt(valid_ctcx);
    return valid ? true: false;
}


/**** add sibling and respect indentation  ****/
void xml_add_next_sibling(xmlNodePtr node, xmlNodePtr new_node) {
    xmlNodePtr lf;
    if(node->prev && node->prev->type == XML_TEXT_NODE) {
        lf = xmlCopyNode(node->prev, 1);
    } else {
        lf = xmlNewText(BAD_CAST "\n    ");
    }

    xmlAddNextSibling(node, new_node);
    xmlAddPrevSibling(new_node, lf);

}

/**** add child and respect indentation  ****/
void xml_add_child(xmlNodePtr parent, xmlNodePtr new_node) {
    xmlNodePtr linefeed;
    if(parent->children &&
       parent->children->type == XML_TEXT_NODE &&
       parent->children != parent->last) {
        // copy the identation of the first waypoint
        linefeed = xmlCopyNode(parent->children, 1);
    } else {
        // or just make 4 space ident if there is no identation
        linefeed = xmlNewText(BAD_CAST "\n    ");
    }


    if(parent->last && parent->last->type == XML_TEXT_NODE) {
        // replace the last identation by the new node,
        xmlNodePtr last_lf = xmlReplaceNode(parent->last, linefeed);
        // add the waypoint node,
        xmlAddChild(parent, new_node);
        // and add back the last identation
        xmlAddChild(parent, last_lf);
    } else {
        // if it's not possible to be clever, just add the nodes
        xmlAddChild(parent, linefeed);
        xmlAddChild(parent, new_node);
    }
}


/* --- HISTORICAL OCAML COMMENTS: editFP.ml ---
 * **************** Editing ONE (single) flight plan *************************
 ------------------------------------------
 * * Dummy flight plan (for map calibration)
 ------------------------------------------
 * * Wrapper checking there is currently no flight plan loaded
 ------------------------------------------
 * Nothing to save
 ------------------------------------------
 * Nothing to close
 ------------------------------------------
 * * Add waypoints as geo references
 ------------------------------------------
 * * Loading a flight plan for edition
 ------------------------------------------
 * * Calibration of chosen image (requires a dummy flight plan)
 ------------------------------------------
 * * Displaying the image in the NW corner
 ------------------------------------------
 * * Open a dummy flight plan
 ------------------------------------------
 * * Dialog to finish calibration
 ------------------------------------------
*/
