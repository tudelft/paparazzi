#ifndef STACKCONTAINER_H
#define STACKCONTAINER_H

#include <QWidget>
#include <QStackedWidget>
#include <QVBoxLayout>
#include "configurable.h"
#include "ac_selector.h"

class StackContainer : public QWidget, public Configurable
{
    Q_OBJECT
public:
    explicit StackContainer(std::function<QWidget*(QString, QWidget*)> constructor, QWidget *parent = nullptr);
    explicit StackContainer(std::function<QWidget*(QString, QWidget*)> constructor,
                            std::function<QWidget*(QString, QWidget*)> alt_constructor,
                            QWidget *parent = nullptr);
    void configure(QDomElement c) { conf = c;}

private:
    void handleNewAC(QString ac_id);
    void removeAC(QString ac_id);

    QMap<QString, QWidget*> viewers_widgets;

    QVBoxLayout* vLayout;
    ACSelector* ac_selector;
    QVBoxLayout* stackLayout;

    QDomElement conf;

    std::function<QWidget*(QString, QWidget*)> constructor;
};

#endif // STACKCONTAINER_H
