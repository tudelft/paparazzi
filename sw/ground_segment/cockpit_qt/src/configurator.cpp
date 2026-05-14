#include "configurator.h"
#include <QtXml>
#include <QFile>
#include <QPushButton>
#include <QSplitter>
#include <QMainWindow>
#include <QTabWidget>
#include <iostream>
#include "pprzmain.h"
#include <QLabel>
#include <iostream>
#include "configurable.h"
#include "widget_utils.h"
#include "gcs_utils.h"
#include "PprzApplication.h"
#if defined(SPEECH_ENABLED)
#include "speaker.h"
#endif

static const char* DEFAULT_WIDTH = "1024";
static const char* DEFAULT_HEIGHT = "600";

Q_LOGGING_CATEGORY(LOG_CONFIG, "config")

QWidget* rec_layout_build(QDomElement &ele, QSplitter* parent, int* size) {
    *size = ele.attribute("size", "10").toInt();
    if(ele.tagName() == "rows" or ele.tagName()=="columns") {
        QString id  = ele.attribute("id" , "");
        QSplitter* splitter = new QSplitter(parent);
        if(id != "") {
            splitter->setObjectName(id);
        }
        splitter->setChildrenCollapsible(false);
        if(ele.tagName()=="rows") {
            splitter->setOrientation(Qt::Vertical);
        } else{
            splitter->setOrientation(Qt::Horizontal);
        }

        QList<int> sizes;
        for(auto child=ele.firstChildElement(); !child.isNull(); child=child.nextSiblingElement()) {
            int s = 0;
            rec_layout_build(child, splitter, &s);
            sizes.append(s);
        }
        splitter->setSizes(sizes);

        if(parent != nullptr) {
            parent->addWidget(splitter);
        } else {
            return splitter;
        }
        
    } else if(ele.tagName() == "notebook" or ele.tagName() == "tabs") {
        QString id  = ele.attribute("id" , "");
        QTabWidget* tabWidget = new QTabWidget(parent);
        if(id != "") {
            tabWidget->setObjectName(id);
        }
        
        for(auto child=ele.firstChildElement(); !child.isNull(); child=child.nextSiblingElement()) {
            int s = 0;
            QWidget* page = rec_layout_build(child, nullptr, &s);
            if (page) {
                QString name = child.attribute("title", child.attribute("name", "Tab"));
                tabWidget->addTab(page, name);
            }
        }
        
        if(parent != nullptr) {
            parent->addWidget(tabWidget);
        } else {
            return tabWidget;
        }

    } else if(ele.tagName()=="widget") {
        QString name = ele.attribute("name", "");
        auto container = ele.attribute("container", "stack");
        if(name == "") {
            qCritical(LOG_CONFIG)  << "Invalid widget name";
            exit(-1);
        }

        QString alt = ele.attribute("alt", "");
        QString id  = ele.attribute("id" , name);

        QWidget* widget = makeWidget(parent, container, name, alt);
        widget->setObjectName(id);

        for(auto layout_ele=ele.firstChildElement(); !layout_ele.isNull(); layout_ele=layout_ele.nextSiblingElement()) {
            if(layout_ele.tagName() == "configure") {
                Configurable* c = dynamic_cast<Configurable*>(widget);
                if(c != nullptr) {
                    c->configure(layout_ele);
                } else {
                    qCritical(LOG_CONFIG)  << "Class does not inherit from Configurable!!!";
                    exit(-1);
                }
            } else {
                qCritical(LOG_CONFIG)  << "Unhandled tag " + layout_ele.tagName();
                exit(-1);
            }
        }

        if(parent != nullptr) {
            parent->addWidget(widget);
        } else {
            return widget;
        }
    } else {
        qCritical(LOG_CONFIG)  << "Invalid tag: the tag should be one of [rows, columns, widget]";
        exit(-1);
    }
    return nullptr;
}

void configure_speech(QDomElement ele) {
    (void)ele;
#if defined(SPEECH_ENABLED)
    if(ele.isNull()) {
        return;
    }

    if(ele.hasAttribute("locale")) {
        auto locale = ele.attribute("locale");
        pprzApp()->toolbox()->speaker()->setLocale(locale);
    }

    for(auto m=ele.firstChildElement(); !m.isNull(); m=m.nextSiblingElement()) {
        bool ok;
        auto name = m.attribute("name");
        auto text = m.attribute("text");
        auto timeout = m.attribute("timeout").toInt(&ok);
        if(!ok) {continue;}
        auto priority = m.attribute("priority", "0").toInt(&ok);
        if(!ok) {continue;}
        int expire;
        if(m.hasAttribute("expire")) {
            expire = m.attribute("expire").toInt(&ok);
            if(!ok) {continue;}
        } else {
            expire = timeout;
        }
        auto onChangeStr = m.attribute("onChange", "false");
        auto postprocessing = m.attribute("postprocessing", "");
        bool onChange = false;
        if(onChangeStr == "1" || onChangeStr == "true") {
            onChange = true;
        }
        pprzApp()->toolbox()->speaker()->registerMessage(name, text, timeout, priority, expire, onChange, postprocessing);
    }

    if(speech()) {
        pprzApp()->toolbox()->speaker()->addSentence("Welcome to Paparazzi!");
    }
#endif
}

PprzMain* configure(QString filename) {
    QDomDocument xmlLayout;
    QFile f(filename);
    if(!f.open(QFile::ReadOnly | QFile::Text)) {
        qCritical(LOG_CONFIG)  << "Error while loading layout file " << filename;
        exit(-1);
    }

    xmlLayout.setContent(&f);
    f.close();

    QDomElement root = xmlLayout.documentElement();

    // configure layout
    auto layout_ele = root.firstChildElement("layout");
    if(layout_ele.isNull()) {
        qCritical(LOG_CONFIG)  << "Cannot find a \"layout\" element.";
        exit(-1);
    }

    int width = layout_ele.attribute("width", DEFAULT_WIDTH).toInt();
    int height = layout_ele.attribute("height", DEFAULT_HEIGHT).toInt();

    if (layout_ele.childNodes().length() != 1) {
        qCritical(LOG_CONFIG)  << "Root tag \"layout\" expected to have exactly one child, rows, colums or widget.";
        exit(-1);
    }

    QDomElement first_element = layout_ele.firstChildElement();
    int s;
    QWidget* widget = rec_layout_build(first_element, nullptr, &s);

    auto window = pprzApp()->mainWindow();
    window->setupUi(width, height, widget);


    //configure speech
    configure_speech(root.firstChildElement("speech"));


    return window;
}


/* --- HISTORICAL OCAML COMMENTS: papgets.ml ---
 * keep element if save is true or save is false and attrib name is not ac_id
 ------------------------------------------
 * Drag and drop handler for papgets
 ------------------------------------------
 * With the format sent by Messages
 ------------------------------------------
 * XML spec
 ------------------------------------------
 * Add x and y attributes
 ------------------------------------------
*/
