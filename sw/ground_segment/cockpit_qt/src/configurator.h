#ifndef LAYOUT_BUILDER_H
#define LAYOUT_BUILDER_H

#include <stdexcept>
#include <QtXml>
#include <QSplitter>
#include "pprzmain.h"

#define DECLARE_LAYOUT_EXCEPT(a) class a : public layout_exception {\
public:\
explicit a(const std::string &arg) :\
  layout_exception(arg) {}\
};


class layout_exception : public std::runtime_error {
public:
layout_exception(const std::string &arg) : runtime_error(arg){}
};

DECLARE_LAYOUT_EXCEPT(invalid_widget_name)
DECLARE_LAYOUT_EXCEPT(invalid_tag)
DECLARE_LAYOUT_EXCEPT(file_error)
DECLARE_LAYOUT_EXCEPT(invalid_node)
DECLARE_LAYOUT_EXCEPT(unknown_widget)

PprzMain* configure(QString filename);
QWidget* rec_layout_build(QDomNode &node, QSplitter* parent, int* size);


#endif // LAYOUT_BUILDER_H
