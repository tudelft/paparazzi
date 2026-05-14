/****************************************************************************
** Meta object code from reading C++ file 'IvyQtLink.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.2.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../ext/pprzlinkQt/include/pprzlinkQt/IvyQtLink.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'IvyQtLink.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.2.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_pprzlink__IvyQtLink_t {
    const uint offsetsAndSize[6];
    char stringdata0[37];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(offsetof(qt_meta_stringdata_pprzlink__IvyQtLink_t, stringdata0) + ofs), len 
static const qt_meta_stringdata_pprzlink__IvyQtLink_t qt_meta_stringdata_pprzlink__IvyQtLink = {
    {
QT_MOC_LITERAL(0, 19), // "pprzlink::IvyQtLink"
QT_MOC_LITERAL(20, 15), // "serverConnected"
QT_MOC_LITERAL(36, 0) // ""

    },
    "pprzlink::IvyQtLink\0serverConnected\0"
    ""
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_pprzlink__IvyQtLink[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
       1,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    0,   20,    2, 0x06,    1 /* Public */,

 // signals: parameters
    QMetaType::Void,

       0        // eod
};

void pprzlink::IvyQtLink::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<IvyQtLink *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->serverConnected(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (IvyQtLink::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&IvyQtLink::serverConnected)) {
                *result = 0;
                return;
            }
        }
    }
    (void)_a;
}

const QMetaObject pprzlink::IvyQtLink::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_pprzlink__IvyQtLink.offsetsAndSize,
    qt_meta_data_pprzlink__IvyQtLink,
    qt_static_metacall,
    nullptr,
qt_incomplete_metaTypeArray<qt_meta_stringdata_pprzlink__IvyQtLink_t
, QtPrivate::TypeAndForceComplete<IvyQtLink, std::true_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>



>,
    nullptr
} };


const QMetaObject *pprzlink::IvyQtLink::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *pprzlink::IvyQtLink::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_pprzlink__IvyQtLink.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int pprzlink::IvyQtLink::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 1)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 1;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 1)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 1;
    }
    return _id;
}

// SIGNAL 0
void pprzlink::IvyQtLink::serverConnected()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
