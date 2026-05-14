#ifndef LISTCONTAINER_H
#define LISTCONTAINER_H

#include <QWidget>
#include <QLayout>
#include "configurable.h"
#include "strip.h"

class ListContainer : public QScrollArea, public Configurable
{
    Q_OBJECT
public:
    explicit ListContainer(std::function<QWidget*(QString, QWidget*)> constructor,std::function<QWidget*(QString, QWidget*)> alt_constructor, QWidget *parent = nullptr);
    explicit ListContainer(std::function<QWidget*(QString, QWidget*)> constructor, QWidget *parent = nullptr);
    void configure(QDomElement c) override {conf = c;}

signals:

private:
    void handleNewAC(QString ac_id);
    void removeAC(QString ac_id);

    std::function<QWidget*(QString, QWidget*)> constructor;
    std::function<QWidget*(QString, QWidget*)> alt_constructor;

    QMap<QString, QWidget*> widgets;
    QMap<QString, QWidget*> alt_widgets;

    QMap<QWidget*, QString> buttons;
    QGridLayout* grid_layout;

    QDomElement conf;

public slots:
};

#endif // LISTCONTAINER_H
