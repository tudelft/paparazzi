#include "layer_combo.h"
#include "gcs_utils.h"

LayerCombo::LayerCombo(QWidget *parent) : QWidget(parent), moved_layer_control(nullptr)
{
    scroll = new QScrollArea(this);
    scroll_content = new QWidget(scroll);
    content_layout = new QVBoxLayout(scroll_content);
    main_layout  = new QVBoxLayout(this);
    main_layout->addWidget(scroll);
    scroll->setWidget(scroll_content);
    scroll->setWidgetResizable(true);
    scroll->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);

    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    moved_thumbnail = new QLabel();
    moved_thumbnail->setWindowFlag(Qt::SubWindow);
    moved_thumbnail->setWindowFlag(Qt::FramelessWindowHint);
    moved_thumbnail->setWindowFlag(Qt::WindowStaysOnTopHint);

    scroll_content->installEventFilter(this);
    //scroll->horizontalScrollBar()->setEnabled(false);
}

bool LayerCombo::eventFilter(QObject *object, QEvent *event)
{

    if (object == scroll_content && event->type() == QEvent::Resize) {
        resize(scroll_content->sizeHint().width() + 40 , size().height());
    }
    return false;
}

void LayerCombo::addLayerControl(MapLayerControl* mlc) {
    mlc->setParent(this);
    content_layout->addWidget(mlc);
    map_layer_controls.append(mlc);
}

void LayerCombo::mousePressEvent(QMouseEvent* e) {
    for(auto &mlc: map_layer_controls) {
        if(mlc->underMouse()) {
            moved_layer_control = mlc;
            press_pos = e->pos();

            QSize pSize = mlc->pixmap().size();
            QSize pixSize = QSize(pSize.width()/2, pSize.height()/2);
            moved_thumbnail->setPixmap(mlc->pixmap().scaled(pixSize));
            moved_thumbnail->raise();

            QPoint globalPos = mapToGlobal(e->pos());
            QPoint pos(globalPos.x()-pixSize.width()/2, globalPos.y()-pixSize.height()/2);
            moved_thumbnail->move(pos);
            moved_thumbnail->show();
        }
    }
}

void LayerCombo::mouseMoveEvent(QMouseEvent* e) {
    (void) e;
    if(moved_layer_control) {
        QPoint globalPos = mapToGlobal(e->pos());
#if QT_VERSION_MAJOR == 5 && QT_VERSION_MINOR < 15
        QSize pixSize = moved_thumbnail->pixmap()->size();
#else
        QSize pixSize = moved_thumbnail->pixmap(Qt::ReturnByValue).size();
#endif
        QPoint pos(globalPos.x()-pixSize.width()/2, globalPos.y()-pixSize.height()/2);
        moved_thumbnail->move(pos);
        int w_y = e->pos().y();

//        double scbv = (static_cast<double>(w_y) / static_cast<double>(scroll->height())) * scroll->verticalScrollBar()->maximum();
//        scroll->verticalScrollBar()->setValue(static_cast<int>(scbv));

        if(w_y < 30) {
            int scroll_value = scroll->verticalScrollBar()->value();
            scroll_value = qMax(scroll_value - w_y, scroll->verticalScrollBar()->minimum());
            scroll->verticalScrollBar()->setValue(scroll_value);
        } else if(scroll->height() - w_y < 30) {
            int scroll_value = scroll->verticalScrollBar()->value();
            scroll_value = qMin(scroll_value + scroll->height() - w_y, scroll->verticalScrollBar()->maximum());
            scroll->verticalScrollBar()->setValue(scroll_value);
        }
    }
}

void LayerCombo::mouseReleaseEvent(QMouseEvent* e) {

    if(moved_layer_control) {
        moved_thumbnail->hide();
        int release_y = scroll_content->mapFrom(this, e->pos()).y();
        int old_z = moved_layer_control->zValue();

        auto it = std::max_element(map_layer_controls.begin(), map_layer_controls.end(),
                                   [](MapLayerControl* lmlc, MapLayerControl* rmlc) {
                                           return lmlc->zValue() < rmlc->zValue();
                            });

        MapLayerControl* top_mlc = nullptr;
        if (it != map_layer_controls.end()) {
            top_mlc = *it;
        }

        map_layer_controls.removeAll(moved_layer_control);

        int new_z = top_mlc->zValue();


        for(auto &mlc: map_layer_controls) {
            int pos_y = mlc->pos().y() + mlc->size().height()/2;
            if(release_y > pos_y) {
                new_z--;
            }
            if(mlc->zValue() > old_z && mlc->zValue() <= new_z) {
                mlc->setZValue(mlc->zValue() - 1);  //go down
            }
            else if(mlc->zValue() < old_z && mlc->zValue() >= new_z) {
                mlc->setZValue(mlc->zValue() + 1);  //go up
            }

        }

        moved_layer_control->setZValue(new_z);
        map_layer_controls.append(moved_layer_control);
        std::sort(map_layer_controls.begin(), map_layer_controls.end(),
                  [](MapLayerControl* lmlc, MapLayerControl* rmlc) {
                        return lmlc->zValue() > rmlc->zValue();
        });

        content_layout->removeWidget(moved_layer_control);
        for(auto &mlc: map_layer_controls) {
            content_layout->removeWidget(mlc);
        }

        for(auto &mlc: map_layer_controls) {
            content_layout->addWidget(mlc);
        }

    }
    moved_layer_control = nullptr;
}



void LayerCombo::makeLayerControl(QString name, bool initialState, int z) {
    QString path = user_or_app_path("pictures/map_thumbnails/" + name + ".png");
    auto settings = getAppSettings();
    QPixmap thumbnail = QPixmap(path);
    if(thumbnail.isNull()) {
        path = user_or_app_path("pictures/map_thumbnails/default.png");
        thumbnail = QPixmap(path);
    }

    MapLayerControl* layer_control = new MapLayerControl(name, thumbnail, initialState, z);

    connect(
        layer_control, &MapLayerControl::showLayer, this,
        [=](bool state) {
            emit showLayer(name, state, layer_control->zValue(), layer_control->opacity());
        }
    );

    connect(
        layer_control, &MapLayerControl::layerOpacityChanged, this,
        [=](qreal opacity) {
            emit layerOpacityChanged(name, opacity);
        }
    );

    connect(
        layer_control, &MapLayerControl::zValueChanged, this,
        [=](int z) {
            emit zValueChanged(name, z);
        }
    );

    addLayerControl(layer_control);
}
