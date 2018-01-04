/****************************************************************************
** Meta object code from reading C++ file 'tdk_sensor.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../tdk_sensor.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'tdk_sensor.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_TDK_Sensor_t {
    QByteArrayData data[5];
    char stringdata0[92];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_TDK_Sensor_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_TDK_Sensor_t qt_meta_stringdata_TDK_Sensor = {
    {
QT_MOC_LITERAL(0, 0, 10), // "TDK_Sensor"
QT_MOC_LITERAL(1, 11, 26), // "mf_SignalPointCloudUpdated"
QT_MOC_LITERAL(2, 38, 0), // ""
QT_MOC_LITERAL(3, 39, 26), // "mf_SignalFlagFilterUpdated"
QT_MOC_LITERAL(4, 66, 25) // "mf_SignalFilterBoxUpdated"

    },
    "TDK_Sensor\0mf_SignalPointCloudUpdated\0"
    "\0mf_SignalFlagFilterUpdated\0"
    "mf_SignalFilterBoxUpdated"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_TDK_Sensor[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   29,    2, 0x06 /* Public */,
       3,    0,   30,    2, 0x06 /* Public */,
       4,    0,   31,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void TDK_Sensor::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        TDK_Sensor *_t = static_cast<TDK_Sensor *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->mf_SignalPointCloudUpdated(); break;
        case 1: _t->mf_SignalFlagFilterUpdated(); break;
        case 2: _t->mf_SignalFilterBoxUpdated(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (TDK_Sensor::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&TDK_Sensor::mf_SignalPointCloudUpdated)) {
                *result = 0;
                return;
            }
        }
        {
            typedef void (TDK_Sensor::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&TDK_Sensor::mf_SignalFlagFilterUpdated)) {
                *result = 1;
                return;
            }
        }
        {
            typedef void (TDK_Sensor::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&TDK_Sensor::mf_SignalFilterBoxUpdated)) {
                *result = 2;
                return;
            }
        }
    }
    Q_UNUSED(_a);
}

const QMetaObject TDK_Sensor::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_TDK_Sensor.data,
      qt_meta_data_TDK_Sensor,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *TDK_Sensor::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *TDK_Sensor::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_TDK_Sensor.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int TDK_Sensor::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 3)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 3;
    }
    return _id;
}

// SIGNAL 0
void TDK_Sensor::mf_SignalPointCloudUpdated()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void TDK_Sensor::mf_SignalFlagFilterUpdated()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}

// SIGNAL 2
void TDK_Sensor::mf_SignalFilterBoxUpdated()
{
    QMetaObject::activate(this, &staticMetaObject, 2, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
