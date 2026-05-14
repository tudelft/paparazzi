#include "grid_viewer.h"
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include "dispatcher_ui.h"

GridViewer::GridViewer(QString ac_id, QWidget *parent) : QWidget(parent), ac_id(ac_id) {
    auto grid_layout = new QGridLayout(this);

    auto slam_label = new QLabel("SLAM Grid", this);
    auto slam_button = new QPushButton("ON", this);

    auto obstacle_label = new QLabel("Obstacle Viewer", this);
    auto obstacle_button = new QPushButton("OFF", this);

    grid_layout->addWidget(slam_label, 0, 0);
    grid_layout->addWidget(slam_button, 0, 1);

    grid_layout->addWidget(obstacle_label, 1, 0);
    grid_layout->addWidget(obstacle_button, 1, 1);

    connect(slam_button, &QPushButton::clicked, this, [=]() {
        bool visible = (slam_button->text() == "ON");
        slam_button->setText(visible ? "OFF" : "ON");
        emit DispatcherUi::get()->slamGridVisibilityChanged(!visible);
    });

    connect(obstacle_button, &QPushButton::clicked, this, [=]() {
        bool visible = (obstacle_button->text() == "ON");
        obstacle_button->setText(visible ? "OFF" : "ON");
        emit DispatcherUi::get()->obstacleVisibilityChanged(!visible);
    });
}
