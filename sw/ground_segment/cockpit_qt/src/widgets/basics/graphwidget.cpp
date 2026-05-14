#include "graphwidget.h"
#include <QPainter>
#include <QPaintEvent>
#include <math.h>

GraphWidget::GraphWidget(QWidget *parent) : QWidget(parent),
    history(30000), val_last_move(0), moving(false)
{
    params.autoscale = true;
    setMouseTracking(true);
}

void GraphWidget::pushData(double d) {
    d *= params.scale;
    // remove old data
    auto now = QTime::currentTime();
    while(!data.isEmpty() && data.first().time.msecsTo(now) > history) {
        data.removeFirst();
    }

    if(data.isEmpty() && params.autoscale) {
        params.min = d;
        params.max = d;
    }

    data.append({d, QTime::currentTime()});

    if(params.autoscale) {
        auto [imin, imax] = std::minmax_element(data.begin(), data.end(), [](auto l, auto r){ return l.value < r.value;});

        double delta = imax->value-imin->value;

        auto min = imin->value - 0.05*delta;
        auto max = imax->value + 0.05*delta;

        params.min = qMin(params.min, min);
        params.max = qMax(params.max, max);
    }

    update();
}

void GraphWidget::clearData() {
    data.clear();
}

const int Y_MARGIN = 30;
const int NB_TIME_SCALE = 4;
const int MIN_SCALE_SIZE = 20;

void GraphWidget::paintEvent(QPaintEvent *e) {
    QPainter p(this);
    auto graph_rect = QRect(0, 0, e->rect().width()-1, e->rect().height()-Y_MARGIN-1);

    double min;// = params.min;
    double max;// = params.max;
    double step;
    double delta = params.max-params.min;

    //if(params.autoscale || true) {
    double scale = log10(delta);
    double d = pow(10, floor(scale));
    if(delta < 2*d) {
        step = d/5.0;
    } else if(delta < 5*d) {
        step = d/2.0;
    } else {
        step = d;
    }

    if(step < 1e-5) {
        min = params.min - 1;
        max = params.max + 1;
        //step = 1;
    } else {
        min = params.min - fmod(params.min, step) - step;
        max = params.max + fmod(params.max, step);
    }
    //}




    auto y_of_val = [=](double val){return graph_rect.bottom() - ((val - min)*graph_rect.height())/(max-min);};
    val_of_y = [=](int y){return ((graph_rect.bottom()-y)*(max-min)/graph_rect.height()) + min;};

    int max_w = 0;
    QList<int> ylines;
    QFontMetrics fm(p.font());

    // draw y scale
    if(step > 1e-5) {
        // Make sure there is enough space for the scale
        if(graph_rect.height() > MIN_SCALE_SIZE*1.1) {
            // make sure steps are not too small
            while(graph_rect.height()/((max-min)/step+1) < MIN_SCALE_SIZE) {
                step *= 2;
            }
            // draw scale
            for(int i=0; i < (max-min)/step+1; ++i) {
                double tick = min + i*step;
                int y = y_of_val(tick);
                ylines.append(y);
                auto text = QString::number(tick);
                p.drawText(0, y + fm.height()/2, text);
                auto wt = fm.size(Qt::TextSingleLine, text).width();
                max_w = qMax(max_w, wt);
            }
        }


    }

    // set left coordinate of the graph, so it do not collide with the text,
    // and draw graph background
    graph_rect.setLeft(max_w + 5);
    p.setBrush(QColor(0x303030));
    p.drawRect(graph_rect);

    // draw horizontal lines (for vertical scale)
    for(auto y: ylines) {
        p.drawLine(graph_rect.left(), y, graph_rect.right(), y);
    }

    auto now = QTime::currentTime();
    auto x_of_t = [=](int dt){return graph_rect.right() - (graph_rect.width()*dt)/history;};
    auto dt_of_x = [=](int x){return ((graph_rect.right() - x) * history) / graph_rect.width();};

    // draw time scale
    p.setBrush(Qt::NoBrush);
    p.setPen(Qt::black);
    for(int i=0; i<NB_TIME_SCALE; ++i) {
        auto t = i*history/NB_TIME_SCALE;
        //auto text = QString::number(t/1000.0);
        auto text = QString("%1").arg(t/1000.0, 0, 'g', 2);
        auto wt = fm.size(Qt::TextSingleLine, text).width();
        auto x = x_of_t(t);
        auto x_text = qMin(x-wt/2, graph_rect.right()-wt);
        p.drawLine(x, graph_rect.bottom(), x, graph_rect.bottom()+3);
        p.drawText(x_text, graph_rect.bottom() + 20, text);
    }

    // draw data
    auto path = QPainterPath();
    for(int i=0; i<data.size(); ++i) {
        int x = x_of_t(data[i].time.msecsTo(now));
        int y = y_of_val(data[i].value);
        if(i == 0) {
            path.moveTo(x, y);
        } else {
            path.lineTo(x, y);
        }
    }
    p.setBrush(Qt::NoBrush);
    p.setPen(Qt::white);
    p.drawPath(path);


    if(graph_rect.contains(mousePos)) {
        // draw mouse vertical line
        p.setPen(Qt::red);
        p.drawLine(mousePos.x(), graph_rect.top(), mousePos.x(), graph_rect.bottom());
        //p.drawLine(graph_rect.left(), mousePos.y(), graph_rect.right(), mousePos.y());

        // draw closest point
        auto dt = dt_of_x(mousePos.x());
        int diff_best = history;
        double val = 0;
        int dt_best = 0;
        for(int i=0; i<data.size(); ++i) {
            auto dti = data[i].time.msecsTo(now);
            auto diff=abs(dti-dt);
            if(diff < diff_best) {
                dt_best = dti;
                val = data[i].value;
                diff_best = diff;
            }
        }
        auto x = x_of_t(dt_best);
        auto y = y_of_val(val);
        p.setPen(Qt::white);
        p.setBrush(Qt::white);
        p.drawEllipse(x-5, y-5, 10, 10);

        // draw closest point value
        auto valtxt = QString::number(val);
        auto valtxt_size = fm.size(Qt::TextSingleLine, valtxt);
        auto pos_txt = QPoint(x + 10, y + valtxt_size.height());
        if(pos_txt.x() + valtxt_size.width() > graph_rect.right()) {
            pos_txt.setX(x - valtxt_size.width() - 10);
        }
        if(pos_txt.y() > graph_rect.bottom()) {
            pos_txt.setY(y - 10);
        }
        p.drawText(pos_txt, valtxt);
    }

}

void GraphWidget::wheelEvent(QWheelEvent *event) {
    wheelAccumulator += event->angleDelta().y();
    if(qAbs(wheelAccumulator) < 120) {
        return;
    }

    params.autoscale = false;
    emit autoscaleChanged(params.autoscale);
#if QT_VERSION_MAJOR == 5 && QT_VERSION_MINOR < 15
    double val = val_of_y(event->y());
#else
    double val = val_of_y(event->position().y());
#endif
    double s = wheelAccumulator < 0 ? 2: 1/2.0;
    params.max = val + (params.max-val)*s;
    params.min = val + (params.min-val)*s;

    wheelAccumulator = 0;
}

void GraphWidget::mousePressEvent(QMouseEvent *event) {
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    val_last_move = val_of_y(event->y());
#else
    val_last_move = val_of_y(event->position().y());
#endif
    moving = true;
}

void GraphWidget::mouseReleaseEvent(QMouseEvent *event) {
    (void)event;
    moving = false;
}

void GraphWidget::mouseDoubleClickEvent(QMouseEvent *event) {
    (void)event;
    if(!data.isEmpty()) {
        auto [imin, imax] = std::minmax_element(data.begin(), data.end(), [](auto l, auto r){ return l.value < r.value;});
        double delta = imax->value-imin->value;
        params.min = imin->value - 0.05*delta;
        params.max = imax->value + 0.05*delta;
        update();
    }
}

void GraphWidget::mouseMoveEvent(QMouseEvent *event) {
    if(moving) {
        params.autoscale = false;
        emit autoscaleChanged(params.autoscale);
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
        double val_current = val_of_y(event->y());
#else
        double val_current = val_of_y(event->position().y());
#endif
        params.max += (val_last_move - val_current);
        params.min += (val_last_move - val_current);
    }
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    mousePos = event->pos();
#else
    mousePos = event->position().toPoint();
#endif
    update();
}

void GraphWidget::leaveEvent(QEvent *event) {
    (void)event;
    mousePos = QPoint(0, 0);
    update();
}

QSize GraphWidget::minimumSizeHint() const
{
    //return minSize;
    return QSize(400, 200);
}
