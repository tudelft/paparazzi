#include <QPainter>
#include <QPixmap>
#include <QtXml>
#include <QFile>
#include <QtSvg/QSvgRenderer>
#include <QRegularExpression>
#include "graphics_aircraft.h"

void SetAttrRecur(QDomElement &elem, QString ac_color, QString target_stroke);

GraphicsAircraft::GraphicsAircraft(PprzPalette palette, QString icon_path, int size, QObject *parent) :
    GraphicsObject(palette, parent),
    size(size) //    pixmap(size, size)
{
    loadSvg(icon_path);
    changeColor(palette.getColor());
}

QRectF GraphicsAircraft::boundingRect() const {
    return QRectF(-size/2*scale_factor, -size/2*scale_factor, size*scale_factor, size*scale_factor);
}

void GraphicsAircraft::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    (void)option;
    (void)widget;
    painter->setRenderHint(QPainter::Antialiasing);
    painter->drawPixmap(QRect(-size/2*scale_factor, -size/2*scale_factor, size*scale_factor, size*scale_factor), pixmap);
}

void GraphicsAircraft::changeFocus() {
    update();
}

void GraphicsAircraft::loadSvg(QString path) {
    QFile file(path);
    (void)file.open(QIODevice::ReadOnly);
    QByteArray baData = file.readAll();
    // load svg contents to xml document and edit contents
    svgdoc.setContent(baData);
}

void GraphicsAircraft::changeColor(QColor color) {
    auto el = svgdoc.documentElement();
    // recursively change color
    // calculate proper stroke width for outline based on SVG size
    QSvgRenderer preRenderer(svgdoc.toByteArray());
    double svgWidth = preRenderer.defaultSize().width();
    double targetStroke = 6.0 * (svgWidth / (double)size);
    SetAttrRecur(el, color.name(), QString::number(targetStroke));

    // create svg renderer with edited contents
    QSvgRenderer svgRenderer(svgdoc.toByteArray());

    // create pixmap target (could be a QImage)
    QPixmap pix(svgRenderer.defaultSize());
    pix.fill(Qt::transparent);
    // create painter to act over pixmap
    QPainter pixPainter(&pix);
    pixPainter.setRenderHint(QPainter::Antialiasing);
    // use renderer to render over painter which paints on pixmap
    svgRenderer.render(&pixPainter);
    pixmap = pix;
}

void SetAttrRecur(QDomElement &elem, QString ac_color, QString target_stroke)
{
    if(elem.attribute("pprz") == "ac_color") {
        elem.setAttribute("fill", ac_color);
    }
    
    // adjust stroke-width for elements with black stroke
    if (elem.hasAttribute("stroke")) {
        QString stroke = elem.attribute("stroke").toLower();
        if (stroke == "#000000" || stroke == "black") {
            elem.setAttribute("stroke-width", target_stroke);
        }
    }
    if (elem.hasAttribute("style")) {
        QString style = elem.attribute("style");
        QString styleLower = style.toLower();
        if (styleLower.contains("stroke:#000000") || styleLower.contains("stroke:black")) {
            QRegularExpression rx("stroke-width:[^;]+");
            if (style.contains(rx)) {
                style.replace(rx, "stroke-width:" + target_stroke);
            } else {
                // ensure we append properly if style doesn't end with a semicolon
                if (!style.endsWith(";") && !style.isEmpty()) {
                    style += ";";
                }
                style += "stroke-width:" + target_stroke;
            }
            elem.setAttribute("style", style);
        }
    }

    // loop all children
    for (int i = 0; i < elem.childNodes().count(); i++)
    {
        if (elem.childNodes().at(i).isElement())
        {
            auto el = elem.childNodes().at(i).toElement();
            SetAttrRecur(el, ac_color, target_stroke);
        }
    }
}
