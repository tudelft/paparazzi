#include "tileproviderconfig.h"
#include <iostream>
#include "point2dpseudomercator.h"

TileProviderConfig::TileProviderConfig(QDomElement ele) {
    zoomMin = ele.attribute("zoomMin", "0").toInt();
    zoomMax = ele.attribute("zoomMax", "19").toInt();


    if(ele.hasAttribute("xMin")) {
        xMin = ele.attribute("xMin").toDouble();
    } else {
        xMin = Point2DPseudoMercator::getBounds().left();
    }

    if(ele.hasAttribute("xMax")) {
        xMax = ele.attribute("xMax").toDouble();
    } else {
        xMax = Point2DPseudoMercator::getBounds().right();
    }

    // top and bottom are reversed because of the direction of the y axis
    if(ele.hasAttribute("yMin")) {
        yMin = ele.attribute("yMin").toDouble();
    } else {
        yMin = Point2DPseudoMercator::getBounds().top();
    }

    if(ele.hasAttribute("yMax")) {
        yMax = ele.attribute("yMax").toDouble();
    } else {
        yMax = Point2DPseudoMercator::getBounds().bottom();
    }

    dir = ele.attribute("dir");
    addr = ele.attribute("addr");

    name = ele.attribute("name");

    tileSize = ele.attribute("tileSize").toInt();
    format = ele.attribute("format");
}


void TileProviderConfig::printConfig() {
    std::cout <<
        "Name: " << name.toStdString() <<
        "  Dir: " << dir.toStdString() <<
        "  Addr: " << addr.toStdString() <<
                 std::endl;
}

bool TileProviderConfig::isValid(Point2DTile pt_tile) {
    // check if zoom is in range
    if(pt_tile.zoom() < zoomMin || pt_tile.zoom() > zoomMax) {
        return false;
    }

    Point2DPseudoMercator pm(pt_tile);
    // check is position is in the extent
    return pm.x() <= xMax &&
           pm.x() >= xMin &&
           pm.y() <= yMax &&
           pm.y() >= yMin;
}
