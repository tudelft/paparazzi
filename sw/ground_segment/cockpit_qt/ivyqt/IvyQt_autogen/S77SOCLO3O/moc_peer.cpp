/****************************************************************************
** Meta object code from reading C++ file 'peer.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.2.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../ext/pprzlinkQt/IvyQt/include/IvyQt/peer.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'peer.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.2.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_Peer_t {
    const uint offsetsAndSize[28];
    char stringdata0[109];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(offsetof(qt_meta_stringdata_Peer_t, stringdata0) + ofs), len 
static const qt_meta_stringdata_Peer_t qt_meta_stringdata_Peer = {
    {
QT_MOC_LITERAL(0, 4), // "Peer"
QT_MOC_LITERAL(5, 7), // "message"
QT_MOC_LITERAL(13, 0), // ""
QT_MOC_LITERAL(14, 2), // "id"
QT_MOC_LITERAL(17, 6), // "params"
QT_MOC_LITERAL(24, 13), // "directMessage"
QT_MOC_LITERAL(38, 10), // "identifier"
QT_MOC_LITERAL(49, 11), // "quitRequest"
QT_MOC_LITERAL(61, 8), // "peerDied"
QT_MOC_LITERAL(70, 5), // "Peer*"
QT_MOC_LITERAL(76, 5), // "ready"
QT_MOC_LITERAL(82, 13), // "pingCompleted"
QT_MOC_LITERAL(96, 7), // "ping_id"
QT_MOC_LITERAL(104, 4) // "msec"

    },
    "Peer\0message\0\0id\0params\0directMessage\0"
    "identifier\0quitRequest\0peerDied\0Peer*\0"
    "ready\0pingCompleted\0ping_id\0msec"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Peer[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       6,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    2,   50,    2, 0x06,    1 /* Public */,
       5,    2,   55,    2, 0x06,    4 /* Public */,
       7,    0,   60,    2, 0x06,    7 /* Public */,
       8,    1,   61,    2, 0x06,    8 /* Public */,
      10,    1,   64,    2, 0x06,   10 /* Public */,
      11,    2,   67,    2, 0x06,   12 /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString, QMetaType::QStringList,    3,    4,
    QMetaType::Void, QMetaType::Int, QMetaType::QString,    6,    4,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 9,    2,
    QMetaType::Void, 0x80000000 | 9,    2,
    QMetaType::Void, QMetaType::Int, QMetaType::LongLong,   12,   13,

       0        // eod
};

void Peer::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<Peer *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->message((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QStringList(*)>(_a[2]))); break;
        case 1: _t->directMessage((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2]))); break;
        case 2: _t->quitRequest(); break;
        case 3: _t->peerDied((*reinterpret_cast< Peer*(*)>(_a[1]))); break;
        case 4: _t->ready((*reinterpret_cast< Peer*(*)>(_a[1]))); break;
        case 5: _t->pingCompleted((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< qint64(*)>(_a[2]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
        case 3:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
            case 0:
                *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType::fromType< Peer* >(); break;
            }
            break;
        case 4:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
            case 0:
                *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType::fromType< Peer* >(); break;
            }
            break;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (Peer::*)(QString , QStringList );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Peer::message)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (Peer::*)(int , QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Peer::directMessage)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (Peer::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Peer::quitRequest)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (Peer::*)(Peer * );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Peer::peerDied)) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (Peer::*)(Peer * );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Peer::ready)) {
                *result = 4;
                return;
            }
        }
        {
            using _t = void (Peer::*)(int , qint64 );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Peer::pingCompleted)) {
                *result = 5;
                return;
            }
        }
    }
}

const QMetaObject Peer::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_Peer.offsetsAndSize,
    qt_meta_data_Peer,
    qt_static_metacall,
    nullptr,
qt_incomplete_metaTypeArray<qt_meta_stringdata_Peer_t
, QtPrivate::TypeAndForceComplete<Peer, std::true_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<QString, std::false_type>, QtPrivate::TypeAndForceComplete<QStringList, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<int, std::false_type>, QtPrivate::TypeAndForceComplete<QString, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<Peer *, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<Peer *, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<int, std::false_type>, QtPrivate::TypeAndForceComplete<qint64, std::false_type>



>,
    nullptr
} };


const QMetaObject *Peer::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Peer::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_Peer.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int Peer::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    }
    return _id;
}

// SIGNAL 0
void Peer::message(QString _t1, QStringList _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void Peer::directMessage(int _t1, QString _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void Peer::quitRequest()
{
    QMetaObject::activate(this, &staticMetaObject, 2, nullptr);
}

// SIGNAL 3
void Peer::peerDied(Peer * _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void Peer::ready(Peer * _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void Peer::pingCompleted(int _t1, qint64 _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
