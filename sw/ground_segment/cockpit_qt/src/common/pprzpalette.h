#ifndef PPRZPALETTE_H
#define PPRZPALETTE_H

#include <QColor>
#include <QBrush>

class PprzPalette
{
public:
    PprzPalette();
    PprzPalette(QColor color);
    PprzPalette(QColor color, QBrush brush);

    QColor getColor() {return main_color;}
    QBrush getBrush() {return main_brush;}
    bool isValid() {return main_color.isValid();}
    QList<QColor> getVariants() {return variants;}
    QColor getVariant(int i) {return variants[i];}

private:
    void makeVariants();

    QColor main_color;
    QBrush main_brush;

    QList<QColor> variants;
};

#endif // PPRZPALETTE_H
