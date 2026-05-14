#include "listcontainer.h"
#include "dispatcher_ui.h"
#include "AircraftManager.h"
#include "strip.h"

ListContainer::ListContainer(std::function<QWidget*(QString, QWidget*)> constructor, QWidget *parent) :
    ListContainer(constructor, nullptr, parent)
{}

ListContainer::ListContainer(std::function<QWidget*(QString, QWidget*)> constructor,
                             std::function<QWidget*(QString, QWidget*)> alt_constructor,
                             QWidget *parent) : QScrollArea(parent),
    constructor(constructor),
    alt_constructor(alt_constructor)
{
    auto scroll_content = new QWidget(this);
    setWidget(scroll_content);
    setWidgetResizable(true);
    setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    auto vbox = new QVBoxLayout(scroll_content);
    grid_layout = new QGridLayout();
    vbox->addLayout(grid_layout);
    vbox->addStretch(1);

    // FIXME: small hack to set minimum size of the scrollarea
    QTimer* t = new QTimer(this);
    connect(t, &QTimer::timeout, this, [=]() {
        this->setMinimumWidth(grid_layout->sizeHint().width());
    });
    t->start(500);

    connect(DispatcherUi::get(), &DispatcherUi::new_ac_config, this, &ListContainer::handleNewAC);
    connect(DispatcherUi::get(), &DispatcherUi::ac_deleted, this, &ListContainer::removeAC);
}

void ListContainer::handleNewAC(QString ac_id) { 
    QWidget* pageWidget;
    try {
        pageWidget = constructor(ac_id, this);
        if(!conf.isNull()) {
            Configurable* c = dynamic_cast<Configurable*>(pageWidget);
            if(c != nullptr) {
                c->configure(conf);
            } else {
                throw std::runtime_error("Widget is not Configurable!!!");
            }
        }
    }  catch (std::runtime_error &e) {
        auto msg = QString(e.what());
        pageWidget = new QWidget(this);
        auto lay = new QVBoxLayout(pageWidget);
        auto label = new QLabel(msg, pageWidget);
        label->setWordWrap(true);
        lay->addWidget(label);
        lay->addStretch();
    }
    widgets[ac_id] = pageWidget;
    auto row = grid_layout->rowCount();
    grid_layout->addWidget(pageWidget, row, 1);


    auto ac = AircraftManager::get()->getAircraft(ac_id);
    auto header = new QWidget(this);
    auto header_vbox = new QVBoxLayout(header);

    auto select_ac_button = new QToolButton(header);
    select_ac_button->setMinimumSize(QSize(20, 20));
    select_ac_button->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);
    select_ac_button->setStyleSheet("background: " + ac->getColor().name());
    header_vbox->addWidget(select_ac_button);
    if(alt_constructor != nullptr) {
        auto action_button = new QToolButton(header);
        action_button->setArrowType(Qt::DownArrow);
        header_vbox->addWidget(action_button);
        QWidget* alt_widget = alt_constructor(ac_id, nullptr);
        alt_widgets[ac_id] = alt_widget;
        alt_widget->setWindowFlags(Qt::FramelessWindowHint | Qt::Popup);

        connect(action_button, &QToolButton::clicked, this,
            [=]() {
                auto p = action_button->mapToGlobal(action_button->rect().bottomRight());
                alt_widget->move(p);
                alt_widget->show();
                auto screen = action_button->screen();
                if (screen) {
                    auto rect_screen = screen->geometry();
                    if(!rect_screen.contains(alt_widget->geometry())) {
                        // the popup widget does not fit in the screen
                        // move it to be contained in the current screen.
                        auto dp = rect_screen.bottomRight() - alt_widget->geometry().bottomRight();
                        dp = QPoint(min(0, dp.x()), min(0, dp.y()));
                        alt_widget->move(p + dp);
                    }
                }

            });
        connect(select_ac_button, &QObject::destroyed, action_button,
                [=]() {
            action_button->deleteLater();
        });
    }

    grid_layout->addWidget(header, row, 0);

    buttons[select_ac_button] = ac_id;

    connect(select_ac_button, &QToolButton::clicked, this,
        [=]() {
            emit DispatcherUi::get()->ac_selected(ac_id);
        });

}

void ListContainer::removeAC(QString ac_id) {
    grid_layout->removeWidget(widgets[ac_id]);
    widgets[ac_id]->deleteLater();
    widgets.remove(ac_id);

    if(alt_widgets.contains(ac_id)) {
        alt_widgets[ac_id]->deleteLater();
        alt_widgets.remove(ac_id);
    }

    auto rect = std::find(buttons.begin(), buttons.end(), ac_id);
    if(rect != buttons.end()) {
        grid_layout->removeWidget(rect.key());
        rect.key()->deleteLater();
        buttons.remove(rect.key());
    }
}
