sed -i -e '/mainToolBar->addAction(toggleMapAction);/a\
            if (navMenu) {\
                for (auto a : navMenu->actions()) {\
                    if (a->text() == "Background") {\
                        a->setCheckable(true);\
                        a->setChecked(toggleMapAction->isChecked());\
                        connect(toggleMapAction, \&QAction::toggled, a, \&QAction::setChecked);\
                        connect(a, \&QAction::toggled, toggleMapAction, \&QAction::setChecked);\
                        break;\
                    }\
                }\
            }' /home/n3yh3hnii/paparazzi/sw/ground_segment/cockpit_qt/src/MainWindow.cpp
