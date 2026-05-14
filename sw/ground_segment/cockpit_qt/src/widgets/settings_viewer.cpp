/*

 * GUI to save settings in the airframe file
 *
 * Copyright (C) 2008, Cyril Allignol, Pascal Brisset
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/*

 * GUI to save settings in the airframe file
 *
 * Copyright (C) 2008, Cyril Allignol, Pascal Brisset
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/*
 * GUI to save settings in the airframe file
 *
 * Copyright (C) 2008, Cyril Allignol, Pascal Brisset
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

#include "settings_viewer.h"
#include "dispatcher_ui.h"
#include "pprz_dispatcher.h"
#include "AircraftManager.h"
#include "double_slider.h"
#include "switch.h"
#include "gcs_utils.h"
#include "strip.h"
#include <QDebug>

SettingsViewer::SettingsViewer(QString ac_id, QWidget *parent) : QWidget(parent), ac_id(ac_id)
{
    main_layout = new QVBoxLayout(this);
    search_layout = new QHBoxLayout();
    path_save_layout = new QHBoxLayout();
    path_layout = new QHBoxLayout();
    main_layout->addItem(search_layout);


    button_home = new QToolButton(this);
    button_home->setText(QString::fromUtf8("\xE2\x8C\x82"));

    button_save = new QToolButton(this);
    button_save->setIcon(style()->standardIcon(QStyle::SP_DialogSaveButton));

    path_layout->addWidget(button_home);
    path = new QStackedWidget(this);
    path_layout->addWidget(path);

    main_layout->addItem(path_save_layout);
    path_save_layout->addItem(path_layout);
    path_save_layout->addStretch();
    path_save_layout->addWidget(button_save);

    scroll = new QScrollArea();
    scroll_content = new QStackedWidget();

    main_layout->addWidget(scroll);
    scroll->setWidget(scroll_content);
    scroll->setWidgetResizable(true);

    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
//    scroll->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);

    search_bar = new QLineEdit(this);
    search_bar->setPlaceholderText("search");
    search_bar->setClearButtonEnabled(true);
    search_layout->addWidget(search_bar);

    auto search_callback = [=](const QString str) {
        if(str == "") {
            restore_searched_items();
            scroll_content->setCurrentIndex(last_widget_index);
            path->setCurrentIndex(last_path_index);
        } else {
            populate_search_results(str);
            scroll_content->setCurrentIndex(search_result_index);
            path->setCurrentIndex(search_path_index);
        }
    };

    connect(
        search_bar, &QLineEdit::textChanged,
        search_callback
    );

    connect(
        search_bar, &QLineEdit::returnPressed,
        [=]() {
            auto str = search_bar->text();
            search_callback(str);
    }
    );

    //setStyleSheet("QWidget{background-color: #31363b;} QLabel{color:white;} QAbstractButton{color:white;} QLineEdit{color:white;}");
    //setAutoFillBackground(true);

    connect(DispatcherUi::get(), &DispatcherUi::settingUpdated, this, &SettingsViewer::updateSettings);

    init(ac_id);
}

bool SettingsViewer::eventFilter(QObject *object, QEvent *event)
{
    (void)object;
    (void)event;
    if(search_bar->hasFocus() && event->type() == QEvent::KeyPress) {
        return true;    //interrupt event
    }
    return false;
}

void SettingsViewer::init(QString ac_id) {
    this->ac_id = ac_id;
    auto settings = AircraftManager::get()->getAircraft(ac_id)->getSettingMenu();
    create_widgets(settings, QList<SettingMenu*>());
    last_widget_index = widgets_indexes[settings];
    last_path_index = path_indexes[settings];

    auto result_widget = new QWidget(scroll_content);
    auto result_layout = new QVBoxLayout(result_widget);
    result_layout->addStretch();
    search_result_index = scroll_content->addWidget(result_widget);
    auto search_label = new QLabel("  searching...", path);
    search_path_index = path->addWidget(search_label);


    scroll_content->setCurrentIndex(widgets_indexes[settings]);

    connect(
        button_home, &QToolButton::clicked, this,
        [=]() {
            scroll_content->setCurrentIndex(widgets_indexes[settings]);
            path->setCurrentIndex(path_indexes[settings]);
            scroll->verticalScrollBar()->setValue(0);
            search_bar->clear();
        }
    );

    connect(
        button_save, &QToolButton::clicked, this,
        [=]() {
            auto saver_dialog = new SettingSaver(ac_id, this);
            saver_dialog->open();
        }
    );



}

void SettingsViewer::create_widgets(SettingMenu* setting_menu, QList<SettingMenu*> stack) {
    auto widget = new QWidget(scroll_content);
    auto path_widget = new QWidget(this);
    auto menu_layout = new QVBoxLayout(widget);
    auto current_path_layout = new QHBoxLayout(path_widget);

    auto new_stack = QList<SettingMenu*>(stack);
    new_stack.append(setting_menu);

    if(new_stack.size()>0) {
        for(auto setmm = std::next(new_stack.begin()); setmm != new_stack.end(); ++setmm) {
            auto setm = (*setmm);
            auto sep = new QLabel(">", path_widget);
            auto button = new QPushButton(setm->getName(), path_widget);
            sep->setSizePolicy(QSizePolicy::Fixed,QSizePolicy::Fixed);
            button->setSizePolicy(QSizePolicy::Fixed,QSizePolicy::Fixed);
            current_path_layout->addWidget(sep);
            current_path_layout->addWidget(button);

            connect(
                button, &QPushButton::clicked, this,
                [=]() {

                    scroll_content->setCurrentIndex(widgets_indexes[setm]);
                    path->setCurrentIndex(path_indexes[setm]);
                    restore_searched_items();
                    last_widget_index = widgets_indexes[setm];
                    last_path_index = path_indexes[setm];
                    scroll->verticalScrollBar()->setValue(0);
                }
            );
        }
        current_path_layout->addStretch();
    }

    int index_path = path->addWidget(path_widget);
    path_indexes[setting_menu] = index_path;

    for(auto sets: setting_menu->getSettingMenus()) {
        auto button = new QPushButton(sets->getName());
        menu_layout->addWidget(button);

        setting_menu_widgets[sets] = button;

        create_widgets(sets, new_stack);
        connect(
            button, &QPushButton::clicked, this,
            [=]() {
                scroll_content->setCurrentIndex(widgets_indexes[sets]);
                path->setCurrentIndex(path_indexes[sets]);
                restore_searched_items();
                last_widget_index = widgets_indexes[sets];
                last_path_index = path_indexes[sets];
                scroll->verticalScrollBar()->setValue(0);
            }
        );
    }
    for(auto &set: setting_menu->getSettings()) {
        auto setting_widget = makeSettingWidget(set, widget);
        menu_layout->addWidget(setting_widget);
        setting_widgets[set] = setting_widget;
    }
    menu_layout->addStretch();
    int index_widget = scroll_content->addWidget(widget);
    widgets_indexes[setting_menu] = index_widget;
}

void SettingsViewer::restore_searched_items() {
    while (pos_hist.size() > 0) {
        auto pos = pos_hist.takeLast();
        pos.layout->insertWidget(pos.index, pos.widget);
    }
}

void SettingsViewer::populate_search_results(QString searched) {
    restore_searched_items();

    QBoxLayout* l = dynamic_cast<QBoxLayout*>(scroll_content->widget(search_result_index)->layout());

    // search in Menus
    for(auto sets: setting_menu_widgets.keys()) {
        QString name = sets->getName();
        if(name.toLower().contains(searched.toLower())) {
            auto w = setting_menu_widgets[sets];
            int index = w->parentWidget()->layout()->indexOf(w);
            pos_hist.push_back(PositionHistory{w, dynamic_cast<QBoxLayout*>(w->parentWidget()->layout()), index});
            l->insertWidget(l->count()-1, w);
        }
    }

    //search in settings
    for(auto set: setting_widgets.keys()) {
        QString name = set->getName();
        if(name.toLower().contains(searched.toLower())) {
            auto w = setting_widgets[set];
            int index = w->parentWidget()->layout()->indexOf(w);
            pos_hist.push_back(PositionHistory{w, dynamic_cast<QBoxLayout*>(w->parentWidget()->layout()), index});
            l->insertWidget(l->count()-1, w);
        }
    }


}



QWidget* SettingsViewer::makeSettingWidget(Setting* setting, QWidget* parent) {
    QWidget* widget = new QWidget(parent);
    QVBoxLayout* vLay = new QVBoxLayout(widget);
    QHBoxLayout* hlay = new QHBoxLayout();
    vLay->addItem(hlay);

    QWidget* label = new QLabel(setting->getName(), widget);
    label->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    hlay->addWidget(label);

    QPushButton* value_btn = new QPushButton("?", widget);
    value_btn->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    hlay->addWidget(value_btn);

    auto ok_btn = new QToolButton(widget);
    ok_btn->setText(QString::fromUtf8("\xE2\x9C\x93"));
    ok_btn->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    ok_btn->setToolTip("commit");
    hlay->addWidget(ok_btn);

    auto undo_btn = new QToolButton(widget);
    undo_btn->setText(QString::fromUtf8("\xE2\x86\xA9"));
    undo_btn->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    undo_btn->setToolTip("undo");
    hlay->addWidget(undo_btn);

    auto reset_btn = new QToolButton(widget);
    reset_btn->setText(QString::fromUtf8("\xE2\x86\xBA"));
    reset_btn->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    reset_btn->setToolTip("reset to initial");
    hlay->addWidget(reset_btn);

    QCheckBox* auto_cb = nullptr;
    if(setting->showAuto()) {
        auto_cb = new QCheckBox("Auto", widget);
        hlay->addWidget(auto_cb);

        connect(auto_cb, &QCheckBox::toggled, this, [=](bool checked) {
            ok_btn->setEnabled(!checked);
            undo_btn->setEnabled(!checked);
        });
    }

    // Auto update if checked
    auto auto_update = [=](float value) {
        if(auto_cb && auto_cb->isChecked()) {
            AircraftManager::get()->getAircraft(ac_id)->setSetting(setting, value);
            setting->setUserValue(value);
        }
    };

    connect(value_btn, &QPushButton::clicked, this, [=]() {
        value_btn->setText("?");
        pprzlink::Message getSetting(PprzDispatcher::get()->getDict()->getDefinition("GET_DL_SETTING"));
        getSetting.addField("ac_id", ac_id);
        getSetting.addField("index", setting->getNo());
        PprzDispatcher::get()->sendMessage(getSetting);
    });

    connect(undo_btn, &QToolButton::clicked, this, [=]() {
        auto prev = setting->getPreviousValue();
        AircraftManager::get()->getAircraft(ac_id)->setSetting(setting, prev);
        setting->setUserValue(prev);
    });

    connect(reset_btn, &QToolButton::clicked, this, [=]() {
        auto initial_value = setting->getInitialValue();
        if(initial_value.has_value()) {
            AircraftManager::get()->getAircraft(ac_id)->setSetting(setting, initial_value.value());
            setting->setUserValue(initial_value.value());
        }
    });

    initialized[setting] = false;

    auto [min, max, step] = setting->getBounds();

    if(setting->getValues().size() > 2) {
        // named values -> combo box
        QComboBox* combo = new QComboBox(widget);
        for(auto &s:setting->getValues()) {
            combo->addItem(s);
        }
        vLay->addWidget(combo);

        setters[setting] = [combo](double value) {combo->setCurrentIndex(static_cast<int>(value));};
        label_setters[setting] = [combo, value_btn](double value) {
            value_btn->setText(combo->itemText(static_cast<int>(value)));
        };

        connect(ok_btn, &QToolButton::clicked, this, [=]() {
            float value = static_cast<float>(combo->currentIndex());
            AircraftManager::get()->getAircraft(ac_id)->setSetting(setting, value);
            setting->setUserValue(value);
        });

        connect(combo, &QComboBox::currentIndexChanged, this, [=](int index) {
            auto_update(static_cast<float>(index));
        });

    } else if(setting->getValues().size() == 2 || abs(min+step-max) < 0.0001) {
        QString s1 = setting->getValues().size() == 2 ? setting->getValues()[0] : QString::number(min);
        QString s2 = setting->getValues().size() == 2 ? setting->getValues()[1] : QString::number(max);
        QHBoxLayout *hbox = new QHBoxLayout;
        Switch* sw = new Switch();
        QLabel* l1 = new QLabel(s1);
        l1->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        QLabel* l2 = new QLabel(s2);
        l2->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        hbox->addWidget(l1);
        hbox->addWidget(sw);
        hbox->addWidget(l2);

        vLay->addLayout(hbox);

        setters[setting] = [sw, min=min, step=step](double value) {sw->setChecked(value > (min + step/2));};
        label_setters[setting] = [setting, value_btn, s1, s2, min=min, step=step, max=max](double value) {
            QString txt;
            if(setting->getValues().size() == 2 && value >= min && value <= max) {
                txt = value < (min + step/2) ? s1: s2;
            } else {
                txt = QString::number(value);
            }

            value_btn->setText(txt);
        };

        connect(ok_btn, &QToolButton::clicked, this, [=, min=min, max=max]() {
            float value = sw->isChecked() ? max : min;
            AircraftManager::get()->getAircraft(ac_id)->setSetting(setting, value);
            setting->setUserValue(value);
        });

        connect(sw, &Switch::toggled, this, [=](bool checked) {
            auto_update(checked ? max : min);
        });

    } else if(abs(max-min) < 0.00001) {
        setters[setting] = [](double value) {(void)value;};
        label_setters[setting] = [value_btn](double value) {value_btn->setText(QString::number(value));};
        float value = min;
        QLabel* uniq_val = new QLabel(QString::number(value));
        uniq_val->setAlignment(Qt::AlignCenter);
        vLay->addWidget(uniq_val);

        connect(ok_btn, &QToolButton::clicked, this, [=]() {
            AircraftManager::get()->getAircraft(ac_id)->setSetting(setting, value);
            setting->setUserValue(value);
        });

    } else {
        DoubleSlider* slider = new DoubleSlider(Qt::Horizontal, widget);
        QLineEdit* raw_edit = new QLineEdit(widget);
        raw_edit->hide();
        QToolButton* expert_button = new QToolButton(widget);
        expert_button->setIcon(QIcon(":/pictures/lock_dark.svg"));
        expert_button->setToolTip("Expert mode");

        setters[setting] = [slider, raw_edit](double value) {slider->setDoubleValue(value); raw_edit->setText(QString::number(value));};
        label_setters[setting] = [value_btn](double value) {value_btn->setText(QString::number(value));};

        slider->setDoubleRange(min, max, step);
        connect(slider, &DoubleSlider::doubleValueChanged, this, [=](double value) {
            auto_update(static_cast<float>(value));
        });

        QHBoxLayout* vbox = new QHBoxLayout();
        QLabel* la = new QLabel(QString::number(min, 'f', 2));
        int precision = 0;
        if(step < 1) {
            precision = static_cast<int>(ceil(abs(log10(step))));
        }
        connect(slider, &DoubleSlider::doubleValueChanged,
            [=](double value) {
                la->setText(QString::number(value, 'f', precision));
            });

        connect(expert_button, &QToolButton::clicked,
            [=]() {
                raw_edit->setVisible(!raw_edit->isVisible());
                if(raw_edit->isVisible()) {
                    expert_button->setIcon(QIcon(":/pictures/lock_warning.svg"));
                } else {
                    expert_button->setIcon(QIcon(":/pictures/lock_dark.svg"));
                }
            });

        connect(raw_edit, &QLineEdit::returnPressed, this,
            [=, min=min, max=max]() {
                auto txt = raw_edit->text();
                bool ok;
                auto value = static_cast<float>(txt.toDouble(&ok));
                if(ok) {
                    if(value > min && value < max) {
                        raw_edit->setStyleSheet("QLineEdit{background-color: #88ff88;}");
                    } else {
                        raw_edit->setStyleSheet("QLineEdit{background-color: #ffd088;}");
                    }
                    AircraftManager::get()->getAircraft(ac_id)->setSetting(setting, value);
                } else {
                    raw_edit->setStyleSheet("QLineEdit{background-color: #ff8888;}");
                }
            });

        connect(raw_edit, &QLineEdit::textChanged,
            [=]() {
                raw_edit->setStyleSheet("QLineEdit{background-color: #ffffff;}");
            });

        vbox->addWidget(la);
        la->setAlignment(Qt::AlignCenter);
        vbox->addWidget(slider);
        vbox->addWidget(raw_edit);
        vbox->addWidget(expert_button);
        vLay->addLayout(vbox);

        connect(ok_btn, &QToolButton::clicked, this, [=]() {
            auto coef = setting->getAltUnitCoef();
            float value = static_cast<float>(slider->doubleValue()) / coef;
            AircraftManager::get()->getAircraft(ac_id)->setSetting(setting, value);
            setting->setUserValue(value);
        });
    }

    // Strip buttons
    for(auto &sb : setting->getStripButtons()) {
       QPushButton* b = new QPushButton(sb->name, widget);
       if(sb->icon != "") {
           QString icon_path = appConfig()->value("GCS_ICONS_PATH").toString() + "/" + sb->icon;
           b->setIcon(QIcon(icon_path));
       }
       auto strip = Strip::getStrip(ac_id);
       if(strip) {
           strip->addWidget(b, sb->group);
           connect(b, &QPushButton::clicked, this, [=, val=sb->value]() {
               AircraftManager::get()->getAircraft(ac_id)->setSetting(setting, val);
               setting->setUserValue(val);
           });
       } else {
           b->deleteLater();
       }
    }

    // Key presses
    for(auto &kp : setting->getKeyPresses()) {
        auto shortcut = new QShortcut(QKeySequence(kp->key), this);
        this->shortcuts.append(shortcut);
        connect(shortcut, &QShortcut::activated, this, [=, val=kp->value]() {
            AircraftManager::get()->getAircraft(ac_id)->setSetting(setting, val);
            setting->setUserValue(val);
        });
    }

    QFrame *line = new QFrame(widget);
    vLay->addWidget(line);
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);

    return widget;
}

void SettingsViewer::updateSettings(QString id, Setting* setting, float value) {
    if(ac_id == id) {
        auto val = setting->getAltUnitCoef() * value;
        label_setters[setting](val);
        setting->setValue(value);
        if(!initialized[setting]) {
            setters[setting](val);
            setting->setInitialValue(value);
            initialized[setting] = true;
        }
    }
}



SettingSaver::SettingSaver(QString ac_id, QWidget *parent) : QDialog(parent),
    ac_id(ac_id)
{

    setWindowTitle("Save Settings");

    auto settings = AircraftManager::get()->getAircraft(ac_id)->getSettingMenu()->getAllSettings();
    auto airframe = AircraftManager::get()->getAircraft(ac_id)->getAirframe();
    auto params = airframe->getParams();

    QStringList unsavable;

    auto tree = new QTreeWidget(this);
    tree->setColumnCount(3);
    tree->setHeaderLabels(QStringList() << "Parameter" << "Airframe Value" << "Setting Value");
    tree->header()->setSectionResizeMode(QHeaderView::ResizeMode::ResizeToContents);

    for(auto setting: settings) {
        auto param_name = setting->getParam();
        if(param_name != "" && setting->getValue().has_value()) {
            bool found = false;
            for(auto &p: params) {
                if(p.name == param_name) {
                    auto item = new QTreeWidgetItem(tree);
                    item->setText(0, param_name);
                    item->setText(1, p.value);
                    auto val = setting->getValue().value() * setting->getAltUnitCoef(p.unit);
                    auto valtxt = QString::number(val);
                    item->setText(2, valtxt);
                    if(abs(val - p.value.toFloat()) > 0.0001) {
                        item->setCheckState(0, Qt::Checked);
                    } else {
                        item->setCheckState(0, Qt::Unchecked);
                    }

                    found = true;
                }
            }
            if(!found) {
                unsavable.append(param_name);
            }
        }
    }

    QString unsavables_text = "Some parameters not writable in the airframe file:\n\n" + unsavable.join("\n");
    QMessageBox::warning(this, tr("Save Settings"), unsavables_text, QMessageBox::Ok);

    auto lay = new QVBoxLayout(this);

    auto all_check = new QCheckBox("select/unselect all", this);

    lay->addWidget(all_check);


    connect(all_check, &QCheckBox::toggled, this, [=](bool state) {
        for(int i=0; i < tree->topLevelItemCount(); i++) {
            auto item = tree->topLevelItem(i);
            item->setCheckState(0, state ? Qt::Checked : Qt::Unchecked);
        }
    });


    lay->addWidget(tree);



    auto buttonBox = new QDialogButtonBox(QDialogButtonBox::Save | QDialogButtonBox::Cancel, this);
    lay->addWidget(buttonBox);

    connect(buttonBox->button(QDialogButtonBox::Save), &QPushButton::clicked, this, [=](){
        QMap<QString, QString> changed_params;

        for(int i=0; i < tree->topLevelItemCount(); i++) {
            auto item = tree->topLevelItem(i);
            if(item->checkState(0)) {
                auto param_name = item->text(0);
                auto value = item->text(2);
                changed_params[param_name] = value;
            }
        }

        auto uri = AircraftManager::get()->getAircraft(ac_id)->getConfig()->getAirframeUri();
        QString separator = "://";
        int sepi = uri.indexOf(separator);
        (void)sepi;
        QString path;
        if(uri.left(sepi) == "file") {
            path = uri.mid(sepi + separator.size());
        } else {
            auto settings = getAppSettings();
            path = appConfig()->value("PAPARAZZI_HOME").toString() + "/conf/airframes";
        }

        auto savePath = QFileDialog::getSaveFileName(this, "Save File", path, "XML files (*.xml)", 0, QFileDialog::DontUseNativeDialog);
        //auto savePath = QFileDialog::getSaveFileName(this, "Save File", path, "XML files (*.xml)", 0);

        if(savePath != "") {
            if(savePath == path) {
                // copy original file before overwriting it
                auto suffix = QDateTime::currentDateTime().toString("_yy_MM_dd__hh_mm_ss") + ".xml";
                auto newpath = path.left(path.length() - 4) + suffix;
                QFile::copy(path, newpath);
            }
            airframe->setParams(changed_params);
            airframe->saveSettings(savePath);
        }

        accept();

    });

    connect(buttonBox->button(QDialogButtonBox::Cancel), &QPushButton::clicked, this, [=](){
        reject();
    });
}


/* --- HISTORICAL OCAML COMMENTS: saveSettings.ml ---
 * * How to have them local ?
 ------------------------------------------
 * * Float not equal to 0.1%
 ------------------------------------------
 * * The save file dialog box
 ------------------------------------------
 * * Toggling a tree element
 ------------------------------------------
 * Connect the column header click to the save_all check button
 ------------------------------------------
 * * Connect the save_all button to all the rows
 ------------------------------------------
 * * Get the scaling between the unit set in the airframe file to the real unit used (code_unit)
       * Print error and use a factor of 1 when code_unit (in airframe file) and unit (in settings file) not equal
 ------------------------------------------
 * raise Invalid_argument
 ------------------------------------------
 * Printf.fprintf stderr "param %s: unit_code=%s unit_airframe=%s\n" param unit_code unit_airframe; flush stderr;
 ------------------------------------------
 * * settings are displayed in alt_unit specified in settings file
       * first try the alt_coef, otherwise try to convert the units
 ------------------------------------------
 * test if is has to be saved as integer or float
 ------------------------------------------
 * Printf.fprintf stderr "param %s: airframe_scale=%f display_scale=%f extra_scale=%f\n" param airframe_scale display_scale extra_scale; flush stderr;
 ------------------------------------------
 * Not savable
 ------------------------------------------
 * Warning if needed
 ------------------------------------------
 * * The popup window displaying airframe and settings values
 ------------------------------------------
 * Build the list window
 ------------------------------------------
 * Build the tree model
 ------------------------------------------
 * * Attach the model to the view
 ------------------------------------------
 * * Render the columns
 ------------------------------------------
 * Parse the airframe file
 ------------------------------------------
 * * Insert the row data in the tree
 ------------------------------------------
 * * The Cancel button
 ------------------------------------------
 * * Connect the Save button to the write action
 ------------------------------------------
 * * Connect the Save button to the write action
 ------------------------------------------
*/
