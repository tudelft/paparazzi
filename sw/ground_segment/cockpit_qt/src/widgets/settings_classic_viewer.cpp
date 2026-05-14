
#include "settings_classic_viewer.h"
#include "AircraftManager.h"
#include "dispatcher_ui.h"
#include "pprz_dispatcher.h"
#include "setting.h"
#include "setting_menu.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QSpinBox>
#include <QSlider>
#include "basics/double_slider.h"
#include <cmath>

#include <QComboBox>
#include <QRadioButton>
#include <QToolButton>
#include <QPushButton>
#include <QCheckBox>
#include <QButtonGroup>
#include <QScrollArea>

SettingsClassicViewer::SettingsClassicViewer(QString ac_id, QWidget *parent)
    : QWidget(parent), ac_id(ac_id)
{
    QVBoxLayout *main_layout = new QVBoxLayout(this);
    main_layout->setContentsMargins(5, 5, 5, 5);
    
    QScrollArea *scroll = new QScrollArea(this);
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    
    QWidget *scroll_content = new QWidget(scroll);
    QVBoxLayout *content_layout = new QVBoxLayout(scroll_content);
    
    // Get raw DOM element from ConfigData
    Aircraft *ac = AircraftManager::get()->getAircraft(ac_id);
    if(ac && ac->getConfig()) {
        QDomDocument doc = ac->getConfig()->getSettings();
        
        auto st_root = doc.firstChildElement( "settings" );
        if(st_root.isNull()) st_root = doc.firstChildElement("generated_settings");
        auto sets = st_root.firstChildElement("dl_settings");
        
        uint8_t setting_no = 0;
        buildSettings(sets, scroll_content, content_layout, setting_no);
    }
    
    content_layout->addStretch();
    scroll->setWidget(scroll_content);
    main_layout->addWidget(scroll);
    
    connect(DispatcherUi::get(), &DispatcherUi::settingUpdated, this, &SettingsClassicViewer::updateSettings);
}

void SettingsClassicViewer::updateSettings(QString id, Setting* setting, float value) {
    if(id == ac_id) {
        uint8_t index = setting->getNo();
        if(value_setters.find(index) != value_setters.end() && !initialized[index]) {
            value_setters[index](value);
            initialized[index] = true;
        }
        if(label_setters.find(index) != label_setters.end()) {
            label_setters[index](value);
        }
    }
}

void SettingsClassicViewer::buildSettings(QDomNode node, QWidget *parentWidget, QBoxLayout *parentLayout, uint8_t& setting_no)
{
    QTabWidget* tabs = nullptr;
    
    for(auto sets = node.firstChildElement(); !sets.isNull(); sets = sets.nextSiblingElement()) {
        if(sets.tagName() == "dl_settings") {
            if(!tabs) {
                tabs = new QTabWidget(parentWidget);
                parentLayout->addWidget(tabs);
            }
            QWidget *tabWidget = new QWidget(tabs);
            QVBoxLayout *tabLayout = new QVBoxLayout(tabWidget);
            tabs->addTab(tabWidget, sets.attribute("name"));
            buildSettings(sets, tabWidget, tabLayout, setting_no);
            tabLayout->addStretch();
        } else if (sets.tagName() == "dl_setting") {
            QWidget *w = makeSettingWidget(sets, parentWidget, setting_no);
            if (w) parentLayout->addWidget(w);
            setting_no++;
        }
    }
}

QWidget* SettingsClassicViewer::makeSettingWidget(QDomElement e, QWidget *parent, uint8_t setting_no)
{
    QWidget *w = new QWidget(parent);
    QHBoxLayout *hlay = new QHBoxLayout(w);
    
    float min = e.attribute("min", "0").toFloat();
    float max = e.attribute("max", "100").toFloat();
    float step = e.attribute("step", "1").toFloat();
    QString varname = e.attribute("var");
    QString shortname = e.attribute("shortname", varname);
    
    float lower = min;
    float upper = max;
    float step_incr = step;
    if(step_incr == 0) step_incr = 1;
    
    QLabel *label = new QLabel(shortname, w);
    label->setMinimumWidth(100);
    hlay->addWidget(label);
    
    QPushButton *curr_val_btn = new QPushButton("?", w);
    curr_val_btn->setMinimumWidth(80);
    hlay->addWidget(curr_val_btn);
    
    QCheckBox *auto_btn = nullptr;
    if(e.attribute("auto") == "true") {
        auto_btn = new QCheckBox("Auto", w);
        hlay->addWidget(auto_btn);
    }
    
    QString widgetType = e.attribute("widget", "auto");
    QString valuesStr = e.attribute("values");
    QStringList values;
    if(!valuesStr.isEmpty()) values = valuesStr.split("|");
    
    bool is_combo = false;
    bool is_radio = false;
    bool is_spin = false;
    bool is_slider = false;
    
    if(widgetType.startsWith("radio", Qt::CaseInsensitive)) is_radio = true;
    else if(widgetType.startsWith("combo", Qt::CaseInsensitive)) is_combo = true;
    else if(widgetType.startsWith("slider", Qt::CaseInsensitive)) is_slider = true;
    else if(widgetType.startsWith("spin", Qt::CaseInsensitive)) is_spin = true;
    else {
        // auto
        if(step_incr == 1.0f && (upper - lower <= 2.0f || values.size() > 0)) {
            if(values.size() > 2) is_combo = true;
            else is_radio = true;
        } else {
            float range = upper - lower;
            if(range > 65536.0f) is_spin = true;
            else is_slider = true;
        }
    }
    
    QWidget *inputWidget = nullptr;
    QButtonGroup *radio_group = nullptr;
    QComboBox *combo = nullptr;
    QDoubleSpinBox *spin = nullptr;
    DoubleSlider *slider = nullptr;
    
    if(is_combo || is_radio) {
        if(is_combo) {
            combo = new QComboBox(w);
            for(int i=0; i<values.size(); ++i) {
                combo->addItem(values[i], lower + i);
            }
            if(values.isEmpty()) {
                for(int i = (int)lower; i <= (int)upper; ++i) combo->addItem(QString::number(i), i);
            }
            inputWidget = combo;
        } else {
            QWidget *radioCont = new QWidget(w);
            QHBoxLayout *radioLay = new QHBoxLayout(radioCont);
            radioLay->setContentsMargins(0,0,0,0);
            radio_group = new QButtonGroup(w);
            int count = upper - lower + 1;
            for(int i=0; i<count; ++i) {
                QString txt = (i < values.size()) ? values[i] : QString::number((int)lower + i);
                QRadioButton *rb = new QRadioButton(txt, radioCont);
                radio_group->addButton(rb, i);
                radioLay->addWidget(rb);
            }
            inputWidget = radioCont;
        }
    } else {
        if(is_spin) {
            spin = new QDoubleSpinBox(w);
            spin->setRange(min, max);
            spin->setSingleStep(step_incr);
            spin->setValue((min+max)/2.0);
            inputWidget = spin;
        } else {
            QWidget* sliderWidget = new QWidget(w);
            QHBoxLayout* sliderLay = new QHBoxLayout(sliderWidget);
            sliderLay->setContentsMargins(0,0,0,0);
            
            int precision = 0;
            if(step_incr < 1) {
                precision = static_cast<int>(ceil(abs(log10(step_incr))));
            }
            
            QLabel* la = new QLabel(QString::number((min+max)/2.0, 'f', precision), sliderWidget);
            sliderLay->addWidget(la);
            
            slider = new DoubleSlider(Qt::Horizontal, sliderWidget);
            slider->setDoubleRange(min, max, step_incr);
            slider->setDoubleValue((min+max)/2.0);
            
            connect(slider, &DoubleSlider::doubleValueChanged, [=](double value) {
                la->setText(QString::number(value, 'f', precision));
            });
            
            sliderLay->addWidget(slider);
            inputWidget = sliderWidget;
        }
    }
    hlay->addWidget(inputWidget);
    
    QToolButton *apply_btn = new QToolButton(w);
    apply_btn->setIcon(qApp->style()->standardIcon(QStyle::SP_DialogApplyButton));
    apply_btn->setToolTip("Commit");
    hlay->addWidget(apply_btn);
    
    QToolButton *undo_btn = new QToolButton(w);
    undo_btn->setIcon(qApp->style()->standardIcon(QStyle::SP_ArrowBack));
    undo_btn->setToolTip("Undo");
    hlay->addWidget(undo_btn);
    
    auto getValueFromUi = [=]() -> float {
        if(combo) return combo->currentData().toFloat();
        if(radio_group) return lower + radio_group->checkedId();
        if(spin) return spin->value();
        if(slider) return slider->doubleValue();
        return 0;
    };
    
    auto apply_cb = [=]() {
        float val = getValueFromUi();
        for(auto set_obj: AircraftManager::get()->getAircraft(ac_id)->getSettingMenu()->getAllSettings()) {
            if(set_obj->getNo() == setting_no) {
                AircraftManager::get()->getAircraft(ac_id)->setSetting(set_obj, val);
                set_obj->setUserValue(val);
                break;
            }
        }
    };
    
    connect(apply_btn, &QToolButton::clicked, apply_cb);

    auto undo_cb = [=]() {
        for(auto set_obj: AircraftManager::get()->getAircraft(ac_id)->getSettingMenu()->getAllSettings()) {
            if(set_obj->getNo() == setting_no) {
                float prev = set_obj->getPreviousValue();
                AircraftManager::get()->getAircraft(ac_id)->setSetting(set_obj, prev);
                set_obj->setUserValue(prev);
                value_setters[setting_no](prev);
                break;
            }
        }
    };
    
    connect(undo_btn, &QToolButton::clicked, undo_cb);
    
    if(auto_btn) {
        connect(auto_btn, &QCheckBox::toggled, [=](bool checked){
            apply_btn->setEnabled(!checked);
            undo_btn->setEnabled(!checked);
            if(checked) apply_cb();
        });
        
        auto try_auto_apply = [=]() {
            if(auto_btn->isChecked()) apply_cb();
        };
        
        if(combo) connect(combo, &QComboBox::currentIndexChanged, try_auto_apply);
        if(radio_group) connect(radio_group, &QButtonGroup::idClicked, try_auto_apply);
        if(spin) connect(spin, &QDoubleSpinBox::editingFinished, try_auto_apply);
        if(slider) connect(slider, &QSlider::sliderReleased, try_auto_apply);
    }
    
    connect(curr_val_btn, &QPushButton::clicked, this, [=]() {
        curr_val_btn->setText("?");
        pprzlink::Message getSetting(PprzDispatcher::get()->getDict()->getDefinition("GET_DL_SETTING"));
        getSetting.addField("ac_id", ac_id);
        getSetting.addField("index", setting_no);
        PprzDispatcher::get()->sendMessage(getSetting);
    });
    
    value_setters[setting_no] = [=](float v) {
        if(combo) {
            int idx = combo->findData(v);
            if(idx >= 0) combo->setCurrentIndex(idx);
        } else if(radio_group) {
            int id = v - lower;
            if(QAbstractButton *rb = radio_group->button(id)) rb->setChecked(true);
        } else if(spin) {
            spin->setValue(v);
        } else if(slider) {
            slider->setDoubleValue(v);
        }
    };
    
    label_setters[setting_no] = [=](float v) {
        if(values.isEmpty()) {
            curr_val_btn->setText(QString::number(v, 'g', 4));
        } else {
            int idx = v - lower;
            if(idx >= 0 && idx < values.size()) {
                curr_val_btn->setText(values[idx]);
            } else {
                curr_val_btn->setText(QString::number(v, 'g', 4));
            }
        }
    };
    
    return w;
}
