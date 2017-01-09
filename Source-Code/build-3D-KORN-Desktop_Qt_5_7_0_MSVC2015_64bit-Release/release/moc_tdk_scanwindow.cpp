/****************************************************************************
** Meta object code from reading C++ file 'tdk_scanwindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../3D-KORN/tdk_scanwindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'tdk_scanwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_TDK_ScanWindow_t {
    QByteArrayData data[10];
    char stringdata0[189];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_TDK_ScanWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_TDK_ScanWindow_t qt_meta_stringdata_TDK_ScanWindow = {
    {
QT_MOC_LITERAL(0, 0, 14), // "TDK_ScanWindow"
QT_MOC_LITERAL(1, 15, 19), // "mf_SlotUpdateWindow"
QT_MOC_LITERAL(2, 35, 0), // ""
QT_MOC_LITERAL(3, 36, 11), // "sensorIndex"
QT_MOC_LITERAL(4, 48, 24), // "mf_SlotUpdateBoundingBox"
QT_MOC_LITERAL(5, 73, 29), // "mf_SlotPointCloudRegistration"
QT_MOC_LITERAL(6, 103, 22), // "flagRegisterDuringScan"
QT_MOC_LITERAL(7, 126, 16), // "mf_SlotStartScan"
QT_MOC_LITERAL(8, 143, 15), // "mf_SlotStopScan"
QT_MOC_LITERAL(9, 159, 29) // "mf_SlotUpdatePointCloudStream"

    },
    "TDK_ScanWindow\0mf_SlotUpdateWindow\0\0"
    "sensorIndex\0mf_SlotUpdateBoundingBox\0"
    "mf_SlotPointCloudRegistration\0"
    "flagRegisterDuringScan\0mf_SlotStartScan\0"
    "mf_SlotStopScan\0mf_SlotUpdatePointCloudStream"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_TDK_ScanWindow[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   44,    2, 0x0a /* Public */,
       4,    0,   47,    2, 0x0a /* Public */,
       5,    1,   48,    2, 0x0a /* Public */,
       7,    0,   51,    2, 0x0a /* Public */,
       8,    0,   52,    2, 0x0a /* Public */,
       9,    0,   53,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,    6,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void TDK_ScanWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        TDK_ScanWindow *_t = static_cast<TDK_ScanWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->mf_SlotUpdateWindow((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->mf_SlotUpdateBoundingBox(); break;
        case 2: _t->mf_SlotPointCloudRegistration((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: _t->mf_SlotStartScan(); break;
        case 4: _t->mf_SlotStopScan(); break;
        case 5: _t->mf_SlotUpdatePointCloudStream(); break;
        default: ;
        }
    }
}

const QMetaObject TDK_ScanWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_TDK_ScanWindow.data,
      qt_meta_data_TDK_ScanWindow,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *TDK_ScanWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *TDK_ScanWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_TDK_ScanWindow.stringdata0))
        return static_cast<void*>(const_cast< TDK_ScanWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int TDK_ScanWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 6)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 6;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
