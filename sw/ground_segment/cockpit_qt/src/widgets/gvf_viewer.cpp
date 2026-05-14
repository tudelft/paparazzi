#include "gvf_viewer.h"
#include "dispatcher_ui.h"

GVFViewer::GVFViewer(QString ac_id, QWidget *parent) : QWidget(parent), ac_id(ac_id)
{   
    auto grid_layout = new QGridLayout(this);
    
    // Widgets/items definitions
    auto traj_vis_label= new QLabel("Trajectory", this);
    auto traj_vis_button = new QPushButton(this);
    traj_vis_button->setText(QString("ON"));

    auto field_vis_label= new QLabel("Vector Field", this);
    auto field_vis_button = new QPushButton(this);
    field_vis_button->setText(QString("ON"));

    auto vspacer1 = new QSpacerItem(1,10);

    auto mode_button = new QPushButton("DEFAULT", this);
    
    auto area_label = new QLabel("Field area", this);
    auto area_spin = new QSpinBox(this); // TODO: Maybe double for small areas??
    area_spin->setMinimum(1);
    area_spin->setMaximum(2147483647);
    area_spin->setValue(2000);

    auto xpts_label = new QLabel("Field Xpts", this);
    auto xpts_spin = new QSpinBox(this);
    xpts_spin->setMinimum(1);
    xpts_spin->setMaximum(100);
    xpts_spin->setValue(24);

    auto ypts_label = new QLabel("Field Ypts", this);
    auto ypts_spin = new QSpinBox(this);
    ypts_spin->setMinimum(1);
    ypts_spin->setMaximum(100);
    ypts_spin->setValue(24);

    auto vspacer2 = new QSpacerItem(1,10);

    auto alt_label = new QLabel("GVF parametric alt (m)", this);
    auto alt_bar = new ColorBar(20, this); 

    // Layout construction
    grid_layout->addWidget(traj_vis_label,   0, 0);
    grid_layout->addWidget(field_vis_label,  1, 0);
    grid_layout->addItem(vspacer1,            2, 0, 1, -1);
    grid_layout->addWidget(mode_button,      3, 0, 1, -1);
    grid_layout->addWidget(area_label,       4, 0);
    grid_layout->addWidget(xpts_label,       5, 0);
    grid_layout->addWidget(ypts_label,       6, 0);
    grid_layout->addItem(vspacer2,           7, 0, 1, -1);
    grid_layout->addWidget(alt_label,        8, 0, 1, -1, Qt::AlignCenter);
    grid_layout->addWidget(alt_bar,          9, 0, 1, -1);

    grid_layout->addWidget(traj_vis_button,  0, 2);
    grid_layout->addWidget(field_vis_button, 1, 2);
    grid_layout->addWidget(area_spin,        4, 2);
    grid_layout->addWidget(xpts_spin,        5, 2);
    grid_layout->addWidget(ypts_spin,        6, 2);

    // GVF Viewer init
    init();

    auto setViewerMode = [=](QString mode) {
        if(mode == "DEFAULT") {
            viewer_mode = "DEFAULT";
            gvfV_config[2] = gvfV_default_Vfield_config[0];
            gvfV_config[3] = gvfV_default_Vfield_config[1];
            gvfV_config[4] = gvfV_default_Vfield_config[2];
            emit DispatcherUi::get()->gvf_settingUpdated(ac_id, &gvfV_config);  

            area_spin->blockSignals(true);
            ypts_spin->blockSignals(true);
            xpts_spin->blockSignals(true);

            area_spin->setValue(gvfV_config[2]);
            xpts_spin->setValue(gvfV_config[3]);
            xpts_spin->setValue(gvfV_config[4]); 

            area_spin->blockSignals(false);
            xpts_spin->blockSignals(false);
            ypts_spin->blockSignals(false);   

        } else if(mode == "CUSTOM") {
            viewer_mode = "CUSTOM";
        }

        mode_button->setText(viewer_mode);
        area_spin->setSingleStep(gvfV_config[2]/10);
    };
    
    setViewerMode("DEFAULT");
    emit DispatcherUi::get()->gvf_settingUpdated(ac_id, &gvfV_config);

    // Building signals-slots connections
    connect(DispatcherUi::get(), &DispatcherUi::gvf_defaultFieldSettings, this,
            [=](QString sender, int area, int xpts, int ypts) {
                if(sender == ac_id) {
                    gvfV_default_Vfield_config[0] = area;
                    gvfV_default_Vfield_config[1] = xpts;
                    gvfV_default_Vfield_config[2] = ypts;
                    setViewerMode(viewer_mode); // if DEFAULT then set def config
                }
            });

    connect(DispatcherUi::get(), &DispatcherUi::gvf_zlimits, this,
            [=](QString sender, float minz, float maxz) {
                if(sender == ac_id) {
                    alt_bar->set_zlimits(minz, maxz);
                    alt_bar->repaint();
                }
            });

    connect(
        mode_button, &QPushButton::clicked, this,
        [=]() {
            if (mode_button->text() == "DEFAULT") {
                setViewerMode("CUSTOM");
            } else {
                setViewerMode("DEFAULT");
            }
        }
    );  

    connect(
        traj_vis_button, &QPushButton::clicked, this,
        [=]() {
            if (traj_vis_button->text() == "ON") {
                gvfV_config[0] = false;
                emit DispatcherUi::get()->gvf_settingUpdated(ac_id, &gvfV_config);
                traj_vis_button->setText(QString("OFF"));
            } else {
                gvfV_config[0] = true;
                emit DispatcherUi::get()->gvf_settingUpdated(ac_id, &gvfV_config);
                traj_vis_button->setText(QString("ON"));
            }
        }
    );

    connect(
        field_vis_button, &QPushButton::clicked, this,
        [=]() {
            if (field_vis_button->text() == "ON") {
                gvfV_config[1] = false;
                emit DispatcherUi::get()->gvf_settingUpdated(ac_id, &gvfV_config);
                field_vis_button->setText(QString("OFF"));
            } else {
                gvfV_config[1] = true;
                emit DispatcherUi::get()->gvf_settingUpdated(ac_id, &gvfV_config);
                field_vis_button->setText(QString("ON"));
            }
        }
    );

    connect(area_spin, qOverload<int>(&QSpinBox::valueChanged), this,
        [=](int value){
            gvfV_config[2] = value;
            setViewerMode("CUSTOM");
        });

    connect(xpts_spin, qOverload<int>(&QSpinBox::valueChanged), this,
        [=](int value){
            gvfV_config[3] = value;
            setViewerMode("CUSTOM");
        });

    connect(ypts_spin, qOverload<int>(&QSpinBox::valueChanged), this,
        [=](int value){
            gvfV_config[4] = value; 
            setViewerMode("CUSTOM");
        });
}

void GVFViewer::init()
{   
    gvfV_config = QVector<int>(5);
    gvfV_default_Vfield_config = QVector<int>(3);
    gvfV_parametric_config = QVector<float>(2);

    gvfV_config[0] = true; //traj_vis
    gvfV_config[1] = true; //field_vis
    gvfV_default_Vfield_config[0] = 2000; //field_area
    gvfV_default_Vfield_config[1] = 24;   //field_xpoints
    gvfV_default_Vfield_config[2] = 24;   //field_ypoints
    gvfV_parametric_config[0] = 0; //minz
    gvfV_parametric_config[1] = 0; //maxz
}