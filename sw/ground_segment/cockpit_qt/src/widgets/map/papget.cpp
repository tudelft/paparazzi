#include "papget.h"
#include <QPainter>
#include <QDebug>
#include <variant>
#include <QApplication>
#include <mapwidget.h>
#include <type_traits>
#include "AircraftManager.h"
#include "papgetconfig.h"
#include "gcs_utils.h"

//TODO must unbind message in destructor, somehow

Papget::Papget(struct DataDef datadef, QPoint pos_view, QObject *parent) : QObject(parent), QGraphicsItem(),
    datadef(datadef), type(Type::NONE), pos_view(pos_view), move_state(MoveState::IDLE)
{
    auto settings = getAppSettings();
    params.style = Style::TEXT;
    auto ac = AircraftManager::get()->getAircraft(datadef.ac_id);
    params.color = ac->getColor();
    params.fontSize = settings.value("map/items_font").toInt();
    data_scale = datadef.scale;

    bindRet = PprzDispatcher::get()->bind(datadef.msg_name,
        [=](QString sender, pprzlink::Message msg) {
            QTimer* timer = new QTimer();
            timer->moveToThread(qApp->thread());
            timer->setSingleShot(true);
            QObject::connect(timer, &QTimer::timeout, this, [=]()
            {
                // main thread
                callback(sender, msg);
                timer->deleteLater();
            });
            QMetaObject::invokeMethod(timer, "start", Qt::QueuedConnection, Q_ARG(int, 0));

        });

}


void Papget::callback(QString sender, pprzlink::Message msg) {
    QString msg_name = msg.getDefinition().getName();
    if(QString(sender) == datadef.ac_id && msg_name == datadef.msg_name) {
        //qDebug() << sender.c_str();
        pprzlink::MessageDefinition msg_def = PprzDispatcher::get()->getDict()->getDefinition(datadef.msg_name);
        pprzlink::MessageField field = msg_def.getField(datadef.field);

// types can be:
//        CHAR,
//        INT8,
//        INT16,
//        INT32,
//        UINT8,
//        UINT16,
//        UINT32,
//        FLOAT,
//        STRING
//  and some array of that types (I think)


        //field.getType().getBaseType().
        //int plop;
        //pprzlink::FieldValue fv;
        //std::any plop;
        //std::variant<uint8_t, uint16_t, QString> plop;
        // TODO: get field value, and type ?
        //msg.getField(datadef.field.toStdString(), plop);



        if( try_this<uint8_t, uint32_t>(msg, val_u32)  ||
            try_this<uint16_t, uint32_t>(msg, val_u32) ||
            try_this<uint32_t, uint32_t>(msg, val_u32)) {

            type = Type::U32;
        }
        else if( try_this<int8_t, int32_t>(msg, val_32)   ||
            try_this<int16_t, int32_t>(msg, val_32)  ||
            try_this<int32_t, int32_t>(msg, val_32)) {

            type = Type::I32;
        }
        else if( try_this<float, double>(msg, val_float)) {
            type = Type::FLOAT;
        }
        else if( try_this<QString, QString>(msg, val_str)) {
            type = Type::STR;
        }
        else if( try_this<char, char>(msg, val_char)) {
            type = Type::CHAR;
        } else {
            qDebug() << "type not handled !";
        }

        prepareGeometryChange();

    }
}

template<typename T, typename U>
bool Papget::try_this(pprzlink::Message msg, U &dst) {
    static_assert(std::is_convertible<T, U>());
    try{
        T value;
        msg.getField(datadef.field, value);
        dst = value;
        return true;
    } catch (std::bad_any_cast &ex) {
        return false;
    }
}


void Papget::updateGraphics(MapWidget* map) {
    scale_factor = 1.0/map->scaleFactor();
    QPointF pos_scene = map->mapToScene(pos_view);
    if(move_state == IDLE) {
        setPos(pos_scene);
    }
    double r = map->getRotation();
    setRotation(-r);

}

void Papget::mousePressEvent(QGraphicsSceneMouseEvent *event){
    //QGraphicsItem::mousePressEvent(event);
    pressPos = QPointF(event->pos().x() * scale(), event->pos().y() * scale());
    move_state = PRESSED;
}

void Papget::mouseMoveEvent(QGraphicsSceneMouseEvent *event) {
    auto settings = getAppSettings();
    if(move_state == PRESSED) {
        QPointF dp = event->pos() - pressPos;
        double d = sqrt(dp.x()*dp.x() + dp.y()*dp.y());
        if(d > settings.value("map/move_hyteresis").toInt()) {
            move_state = MOVED;
        }
    } else if(move_state == MOVED) {
        setPos(event->scenePos() - pressPos);
    }
}
void Papget::mouseReleaseEvent(QGraphicsSceneMouseEvent *event) {
    (void)event;
    move_state = IDLE;
    emit moved(pos());
}
void Papget::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event) {
    (void)event;
    Params old_params = params;
    auto pc = new PapgetConfig(datadef, params);

    connect(pc, &PapgetConfig::paramsChanged, this, [=](Params new_params) mutable {
            params = new_params;
    });

    connect(pc, &QDialog::finished, this, [=](int result) mutable {
        if(!result) {
            params = old_params;
        }
    });

    pc->open();
}

QRectF Papget::boundingRect() const {
    return  bounding_rect;
}

void Papget::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    if(params.style == Style::TEXT) {
        paint_text(painter, option, widget);
    }
}

void Papget::paint_text(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    (void)option;
    (void)widget;

    QString text;
    if(type == U32) {
        text = QString::number(data_scale*val_u32);
    } else if(type == I32) {
        text = QString::number(data_scale*val_32);
    } else if(type == FLOAT) {
        text = QString::number(data_scale*val_float);
    } else if(type == STR) {
        text = QString(val_str);
    } else if(type == CHAR) {
        text = QString(val_char);
    } else if (type == NONE) {
        qDebug() << "Papget: no good value";
    }

    QFont font;
    font.setPointSize (qMax(1, static_cast<int>(params.fontSize*scale_factor)));
    font.setWeight(QFont::DemiBold);
    QPainterPath path_draw;
    path_draw.addText(0, 0, font, text);
    painter->setBrush(params.color);
    painter->setPen(Qt::NoPen);
    painter->drawPath(path_draw);

    QFontMetrics fm(font);

    bounding_rect = fm.boundingRect(text);
}

//QPainterPath Papget::shape() const {
//        return QGraphicsItem::shape();
//}
