#include "widget_utils.h"
#include "listcontainer.h"
#include "ac_selector.h"
#include "pprzmap.h"
#include "pfd.h"
#include "layer_combo.h"
#include "settings_viewer.h"
#include "stackcontainer.h"
#include "flightplan_viewerv2.h"
#include "commands.h"
#include "gps_classic_viewer.h"
#include "flightplaneditor.h"
#include "mini_strip.h"
#include "link_status.h"
#include "tuple_helpers.h"
#include "gvf_viewer.h"
#include "grid_viewer.h"

#include "alert_viewer.h"
#include "infrared_viewer.h"
#include "misc_viewer.h"
#include "chat.h"
#include "checklist.h"

using ac_widgets_list = std::tuple<
    SettingsViewer, MiniStrip,
    Commands, FlightPlanViewerV2,
    GPSClassicViewer, FlightPlanEditor,
    LinkStatus, GVFViewer, GridViewer,
    Checklist, InfraredViewer, MiscViewer
>;
using simple_widgets_list = std::tuple<PprzMap, Pfd, Chat, AlertViewer>;
using containers_list = std::tuple<StackContainer, ListContainer>;

std::map<QString, size_t> AC_WIDGETS_MAP = {
    {"settings", tuple_element_index_v<SettingsViewer, ac_widgets_list>},
    {"strips", tuple_element_index_v<MiniStrip, ac_widgets_list>},
    {"commands", tuple_element_index_v<Commands, ac_widgets_list>},
    {"flightplan", tuple_element_index_v<FlightPlanViewerV2, ac_widgets_list>},
    {"gps_classic_viewer", tuple_element_index_v<GPSClassicViewer, ac_widgets_list>},
    {"flight_plan_editor", tuple_element_index_v<FlightPlanEditor, ac_widgets_list>},
    {"link_status", tuple_element_index_v<LinkStatus, ac_widgets_list>},
    {"link", tuple_element_index_v<LinkStatus, ac_widgets_list>},
    {"gvf_viewer", tuple_element_index_v<GVFViewer, ac_widgets_list>},
    {"checklist", tuple_element_index_v<Checklist, ac_widgets_list>},
    {"grid_viewer", tuple_element_index_v<GridViewer, ac_widgets_list>},
    {"infrared", tuple_element_index_v<InfraredViewer, ac_widgets_list>},
    {"misc", tuple_element_index_v<MiscViewer, ac_widgets_list>},
};


std::map<QString, size_t> SIMPLE_WIDGETS_MAP = {
    {"map2d", tuple_element_index_v<PprzMap, simple_widgets_list>},
    {"PFD", tuple_element_index_v<Pfd, simple_widgets_list>},
    {"chat", tuple_element_index_v<Chat, simple_widgets_list>},
    {"alert", tuple_element_index_v<AlertViewer, simple_widgets_list>},
};



std::map<QString, size_t> CONTAINER_WIDGETS_MAP = {
    {"stack", tuple_element_index_v<StackContainer, containers_list>},
    {"list", tuple_element_index_v<ListContainer, containers_list>},
};

// the following code is inspired by https://codereview.stackexchange.com/a/166731

template <std::size_t... Is, class F>
static inline void static_for_impl(F&& f, std::index_sequence<Is...>) {
    (f(std::integral_constant<std::size_t, Is>()), ...);
}
template <std::size_t N, class F>
void static_for(F f) {
    static_for_impl(f, std::make_index_sequence<N>());
}

template <class tuple, class F>
void for_all_types(F f) {
    static_for<std::tuple_size_v<tuple>>([&](auto N){
        using T = std::tuple_element_t<N, tuple>;
        if constexpr (!std::is_same_v<void, T>)
            f((T*)0, N);
    });
}

template <class R, class tuple, class F>
R select_type(F f, std::size_t i) {
    R r;
    bool found = false;
    for_all_types<tuple>([&](auto p, auto N){
        if (i == N) {
            r = f(p);
            found = true;
        }
    });
    if (!found)
        throw std::invalid_argument("");
    return r;
}



template<class ET, class... X>
auto createInstance(const ET eventType, X&&... x) {
    return select_type<QWidget*, simple_widgets_list>([&](auto p){
        return new std::decay_t<decltype(*p)>(std::forward<X>(x)...);
    }, (std::size_t)eventType);
}


template<class ET>
auto getConstructor(const ET eventType) {
    return select_type<std::function<QWidget*(QString, QWidget*)>, ac_widgets_list>([&](auto p){
        return [](QString ac_id, QWidget* container) {
            return new std::decay_t<decltype(*p)>(ac_id, container);
        };
    }, (std::size_t)eventType);
}


template<class ET, class... X>
auto createContainer(const ET eventType, X&&... x) {
    return select_type<QWidget*, containers_list>([&](auto p){
        return new std::decay_t<decltype(*p)>(std::forward<X>(x)...);
    }, (std::size_t)eventType);
}


QWidget* make_container(QString container, QWidget* parent, size_t wt) {
    auto container_index = CONTAINER_WIDGETS_MAP.find(container);
    if(container_index != CONTAINER_WIDGETS_MAP.end()) {
        return createContainer(container_index->second, getConstructor(wt), parent);
    } else {
        throw runtime_error("unkown container " + container.toStdString());
    }
}

QWidget* make_container(QString container, QWidget* parent, size_t wt, size_t wt2) {
    auto container_index = CONTAINER_WIDGETS_MAP.find(container);
    if(container_index != CONTAINER_WIDGETS_MAP.end()) {
        return createContainer(container_index->second, getConstructor(wt), getConstructor(wt2), parent);
    } else {
        throw runtime_error("unkown container " + container.toStdString());
    }
}


QWidget* makeWidget(QWidget* parent, QString container, QString name, QString alt) {
    QWidget* widget = nullptr;

    auto ac_widget_index = AC_WIDGETS_MAP.find(name);
    auto alt_index = AC_WIDGETS_MAP.find(alt);
    auto simple_widget_index = SIMPLE_WIDGETS_MAP.find(name);

    if(ac_widget_index != AC_WIDGETS_MAP.end()) {
        if(alt_index != AC_WIDGETS_MAP.end()) {
            widget = make_container(container, parent, ac_widget_index->second, alt_index->second);
        } else {
            widget = make_container(container, parent, ac_widget_index->second);
        }

    }
    else if(simple_widget_index != SIMPLE_WIDGETS_MAP.end()) {
        widget = createInstance(simple_widget_index->second, parent);
    }
    else {
        std::string s = "Widget " + name.toStdString() + " unknown";
        throw runtime_error(s);
    }

    return widget;
}


/* --- HISTORICAL OCAML COMMENTS: papget.ml ---
 * * [index_of_fields s] Returns i if s matches x[i] else 0.
 ------------------------------------------
 * get item and window size
 ------------------------------------------
 * delete an item if placed out of the window on the left or top side
 ------------------------------------------
 * Remove the current child ?
 ------------------------------------------
 * Connect the renderer chooser
 ------------------------------------------
 * Connect the buttons
 ------------------------------------------
 * Set the current value
 ------------------------------------------
 * Connect the scale entry
 ------------------------------------------
 * **************************************************************************
 ------------------------------------------
 * * A clickable item is not editable: The #edit method is overiden with a
    provided callback
 ------------------------------------------
 * **************************************************************************
 ------------------------------------------
*/
