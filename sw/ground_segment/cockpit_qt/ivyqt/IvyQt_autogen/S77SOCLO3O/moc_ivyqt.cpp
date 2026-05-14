/****************************************************************************
** Meta object code from reading C++ file 'ivyqt.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.2.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../ext/pprzlinkQt/IvyQt/include/IvyQt/ivyqt.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'ivyqt.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.2.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_IvyQt_t {
    const uint offsetsAndSize[32];
    char stringdata0[138];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(offsetof(qt_meta_stringdata_IvyQt_t, stringdata0) + ofs), len 
static const qt_meta_stringdata_IvyQt_t qt_meta_stringdata_IvyQt = {
    {
QT_MOC_LITERAL(0, 5), // "IvyQt"
QT_MOC_LITERAL(6, 9), // "peerReady"
QT_MOC_LITERAL(16, 0), // ""
QT_MOC_LITERAL(17, 5), // "Peer*"
QT_MOC_LITERAL(23, 4), // "peer"
QT_MOC_LITERAL(28, 13), // "directMessage"
QT_MOC_LITERAL(42, 10), // "identifier"
QT_MOC_LITERAL(53, 6), // "params"
QT_MOC_LITERAL(60, 11), // "quitRequest"
QT_MOC_LITERAL(72, 8), // "peerDied"
QT_MOC_LITERAL(81, 4), // "name"
QT_MOC_LITERAL(86, 7), // "stopped"
QT_MOC_LITERAL(94, 9), // "tcphandle"
QT_MOC_LITERAL(104, 9), // "udphandle"
QT_MOC_LITERAL(114, 10), // "deletePeer"
QT_MOC_LITERAL(125, 12) // "newPeerReady"

    },
    "IvyQt\0peerReady\0\0Peer*\0peer\0directMessage\0"
    "identifier\0params\0quitRequest\0peerDied\0"
    "name\0stopped\0tcphandle\0udphandle\0"
    "deletePeer\0newPeerReady"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_IvyQt[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
       9,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       5,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    1,   68,    2, 0x06,    1 /* Public */,
       5,    3,   71,    2, 0x06,    3 /* Public */,
       8,    1,   78,    2, 0x06,    7 /* Public */,
       9,    1,   81,    2, 0x06,    9 /* Public */,
      11,    0,   84,    2, 0x06,   11 /* Public */,

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
      12,    0,   85,    2, 0x08,   12 /* Private */,
      13,    0,   86,    2, 0x08,   13 /* Private */,
      14,    1,   87,    2, 0x08,   14 /* Private */,
      15,    1,   90,    2, 0x08,   16 /* Private */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 3, QMetaType::Int, QMetaType::QString,    4,    6,    7,
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, QMetaType::QString,   10,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 3,    4,

       0        // eod
};

void IvyQt::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<IvyQt *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->peerReady((*reinterpret_cast< Peer*(*)>(_a[1]))); break;
        case 1: _t->directMessage((*reinterpret_cast< Peer*(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< QString(*)>(_a[3]))); break;
        case 2: _t->quitRequest((*reinterpret_cast< Peer*(*)>(_a[1]))); break;
        case 3: _t->peerDied((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 4: _t->stopped(); break;
        case 5: _t->tcphandle(); break;
        case 6: _t->udphandle(); break;
        case 7: _t->deletePeer((*reinterpret_cast< Peer*(*)>(_a[1]))); break;
        case 8: _t->newPeerReady((*reinterpret_cast< Peer*(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
        case 0:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
            case 0:
                *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType::fromType< Peer* >(); break;
            }
            break;
        case 1:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
            case 0:
                *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType::fromType< Peer* >(); break;
            }
            break;
        case 2:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
            case 0:
                *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType::fromType< Peer* >(); break;
            }
            break;
        case 7:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
            case 0:
                *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType::fromType< Peer* >(); break;
            }
            break;
        case 8:
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
            using _t = void (IvyQt::*)(Peer * );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&IvyQt::peerReady)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (IvyQt::*)(Peer * , int , QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&IvyQt::directMessage)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (IvyQt::*)(Peer * );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&IvyQt::quitRequest)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (IvyQt::*)(QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&IvyQt::peerDied)) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (IvyQt::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&IvyQt::stopped)) {
                *result = 4;
                return;
            }
        }
    }
}

const QMetaObject IvyQt::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_IvyQt.offsetsAndSize,
    qt_meta_data_IvyQt,
    qt_static_metacall,
    nullptr,
qt_incomplete_metaTypeArray<qt_meta_stringdata_IvyQt_t
, QtPrivate::TypeAndForceComplete<IvyQt, std::true_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<Peer *, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<Peer *, std::false_type>, QtPrivate::TypeAndForceComplete<int, std::false_type>, QtPrivate::TypeAndForceComplete<QString, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<Peer *, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<QString, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>
, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<Peer *, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<Peer *, std::false_type>


>,
    nullptr
} };


const QMetaObject *IvyQt::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *IvyQt::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_IvyQt.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int IvyQt::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 9)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 9;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 9)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 9;
    }
    return _id;
}

// SIGNAL 0
void IvyQt::peerReady(Peer * _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void IvyQt::directMessage(Peer * _t1, int _t2, QString _t3)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t3))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void IvyQt::quitRequest(Peer * _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void IvyQt::peerDied(QString _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void IvyQt::stopped()
{
    QMetaObject::activate(this, &staticMetaObject, 4, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
