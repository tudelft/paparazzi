#include "pprzpalette.h"

PprzPalette::PprzPalette()
{

}

PprzPalette::PprzPalette(QColor color): main_color(color), main_brush(color)
{
    makeVariants();
}


PprzPalette::PprzPalette(QColor color, QBrush brush): main_color(color), main_brush(brush)
{
    makeVariants();
}

void PprzPalette::makeVariants() {

    variants.append(main_color);

    int h, s, v, a;
    main_color.getHsv(&h, &s, &v, &a);

    int v1 = qMin(static_cast<int>(v/1.2), 255);
    QColor c1 = QColor(main_color);
    c1.setHsv(h, s, v1, a);
    variants.append(c1);

    int a2 = qMin(static_cast<int>(a/2), 255);
    QColor c2 = QColor(main_color);
    c2.setHsv(h, s, v, a2);
    variants.append(c2);


    int s3 = static_cast<int>(s/2);
    QColor c3 = QColor(main_color);
    //c3.setHsv(h, s3, v, a);
    c3.setHsv(h, s3, v, a2);
    variants.append(c3);
}
