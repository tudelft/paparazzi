#ifndef CONFIGURABLEWIDGET_H
#define CONFIGURABLEWIDGET_H

#include <QtXml>

class Configurable
{
public:
    virtual void configure(QDomElement) = 0;
};

#endif // CONFIGURABLEWIDGET_H
