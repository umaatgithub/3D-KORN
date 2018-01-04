/****************************************************************************
** Meta object code from reading C++ file 'tdk_centralwidget.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../tdk_centralwidget.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'tdk_centralwidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_TDK_CentralWidget_t {
    QByteArrayData data[15];
    char stringdata0[378];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_TDK_CentralWidget_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_TDK_CentralWidget_t qt_meta_stringdata_TDK_CentralWidget = {
    {
QT_MOC_LITERAL(0, 0, 17), // "TDK_CentralWidget"
QT_MOC_LITERAL(1, 18, 30), // "mf_SignalPointCloudListUpdated"
QT_MOC_LITERAL(2, 49, 0), // ""
QT_MOC_LITERAL(3, 50, 40), // "mf_SignalRegisteredPointCloud..."
QT_MOC_LITERAL(4, 91, 24), // "mf_SignalMeshListUpdated"
QT_MOC_LITERAL(5, 116, 25), // "mf_SlotRegisterPointCloud"
QT_MOC_LITERAL(6, 142, 19), // "mf_SlotGenerateMesh"
QT_MOC_LITERAL(7, 162, 30), // "mf_SlotUpdatePointCloudListTab"
QT_MOC_LITERAL(8, 193, 40), // "mf_SlotUpdateRegisteredPointC..."
QT_MOC_LITERAL(9, 234, 24), // "mf_SlotUpdateMeshListTab"
QT_MOC_LITERAL(10, 259, 30), // "mf_SlotUpdatePointCloudDisplay"
QT_MOC_LITERAL(11, 290, 16), // "QListWidgetItem*"
QT_MOC_LITERAL(12, 307, 4), // "item"
QT_MOC_LITERAL(13, 312, 40), // "mf_SlotUpdateRegisteredPointC..."
QT_MOC_LITERAL(14, 353, 24) // "mf_SlotUpdateMeshDisplay"

    },
    "TDK_CentralWidget\0mf_SignalPointCloudListUpdated\0"
    "\0mf_SignalRegisteredPointCloudListUpdated\0"
    "mf_SignalMeshListUpdated\0"
    "mf_SlotRegisterPointCloud\0mf_SlotGenerateMesh\0"
    "mf_SlotUpdatePointCloudListTab\0"
    "mf_SlotUpdateRegisteredPointCloudListTab\0"
    "mf_SlotUpdateMeshListTab\0"
    "mf_SlotUpdatePointCloudDisplay\0"
    "QListWidgetItem*\0item\0"
    "mf_SlotUpdateRegisteredPointCloudDisplay\0"
    "mf_SlotUpdateMeshDisplay"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_TDK_CentralWidget[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      11,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   69,    2, 0x06 /* Public */,
       3,    0,   70,    2, 0x06 /* Public */,
       4,    0,   71,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       5,    0,   72,    2, 0x0a /* Public */,
       6,    0,   73,    2, 0x0a /* Public */,
       7,    0,   74,    2, 0x0a /* Public */,
       8,    0,   75,    2, 0x0a /* Public */,
       9,    0,   76,    2, 0x0a /* Public */,
      10,    1,   77,    2, 0x0a /* Public */,
      13,    1,   80,    2, 0x0a /* Public */,
      14,    1,   83,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 11,   12,
    QMetaType::Void, 0x80000000 | 11,   12,
    QMetaType::Void, 0x80000000 | 11,   12,

       0        // eod
};

void TDK_CentralWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        TDK_CentralWidget *_t = static_cast<TDK_CentralWidget *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->mf_SignalPointCloudListUpdated(); break;
        case 1: _t->mf_SignalRegisteredPointCloudListUpdated(); break;
        case 2: _t->mf_SignalMeshListUpdated(); break;
        case 3: _t->mf_SlotRegisterPointCloud(); break;
        case 4: _t->mf_SlotGenerateMesh(); break;
        case 5: _t->mf_SlotUpdatePointCloudListTab(); break;
        case 6: _t->mf_SlotUpdateRegisteredPointCloudListTab(); break;
        case 7: _t->mf_SlotUpdateMeshListTab(); break;
        case 8: _t->mf_SlotUpdatePointCloudDisplay((*reinterpret_cast< QListWidgetItem*(*)>(_a[1]))); break;
        case 9: _t->mf_SlotUpdateRegisteredPointCloudDisplay((*reinterpret_cast< QListWidgetItem*(*)>(_a[1]))); break;
        case 10: _t->mf_SlotUpdateMeshDisplay((*reinterpret_cast< QListWidgetItem*(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (TDK_CentralWidget::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&TDK_CentralWidget::mf_SignalPointCloudListUpdated)) {
                *result = 0;
                return;
            }
        }
        {
            typedef void (TDK_CentralWidget::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&TDK_CentralWidget::mf_SignalRegisteredPointCloudListUpdated)) {
                *result = 1;
                return;
            }
        }
        {
            typedef void (TDK_CentralWidget::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&TDK_CentralWidget::mf_SignalMeshListUpdated)) {
                *result = 2;
                return;
            }
        }
    }
}

const QMetaObject TDK_CentralWidget::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_TDK_CentralWidget.data,
      qt_meta_data_TDK_CentralWidget,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *TDK_CentralWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *TDK_CentralWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_TDK_CentralWidget.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int TDK_CentralWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 11)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 11;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 11)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 11;
    }
    return _id;
}

// SIGNAL 0
void TDK_CentralWidget::mf_SignalPointCloudListUpdated()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void TDK_CentralWidget::mf_SignalRegisteredPointCloudListUpdated()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}

// SIGNAL 2
void TDK_CentralWidget::mf_SignalMeshListUpdated()
{
    QMetaObject::activate(this, &staticMetaObject, 2, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
