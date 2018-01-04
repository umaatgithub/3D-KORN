/****************************************************************************
** Meta object code from reading C++ file 'tdk_scanwindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../tdk_scanwindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'tdk_scanwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_TDK_ScanWindow_t {
    QByteArrayData data[24];
    char stringdata0[539];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_TDK_ScanWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_TDK_ScanWindow_t qt_meta_stringdata_TDK_ScanWindow = {
    {
QT_MOC_LITERAL(0, 0, 14), // "TDK_ScanWindow"
QT_MOC_LITERAL(1, 15, 22), // "mf_SignalStatusChanged"
QT_MOC_LITERAL(2, 38, 0), // ""
QT_MOC_LITERAL(3, 39, 34), // "mf_SignalDatabasePointCloudUp..."
QT_MOC_LITERAL(4, 74, 44), // "mf_SignalDatabaseRegisteredPo..."
QT_MOC_LITERAL(5, 119, 34), // "mf_SignalNumberOfPointCloudUp..."
QT_MOC_LITERAL(6, 154, 19), // "mf_SlotUpdateWindow"
QT_MOC_LITERAL(7, 174, 11), // "sensorIndex"
QT_MOC_LITERAL(8, 186, 24), // "mf_SlotUpdateBoundingBox"
QT_MOC_LITERAL(9, 211, 24), // "mf_SlotActivateFiltering"
QT_MOC_LITERAL(10, 236, 13), // "flagFiltering"
QT_MOC_LITERAL(11, 250, 29), // "mf_SlotPointCloudRegistration"
QT_MOC_LITERAL(12, 280, 16), // "flagRealTimeScan"
QT_MOC_LITERAL(13, 297, 16), // "mf_SlotStartScan"
QT_MOC_LITERAL(14, 314, 15), // "mf_SlotStopScan"
QT_MOC_LITERAL(15, 330, 31), // "mf_SlotHandlePlatformParameters"
QT_MOC_LITERAL(16, 362, 28), // "flagEnablePlatformParameters"
QT_MOC_LITERAL(17, 391, 29), // "mf_SlotUpdatePointCloudStream"
QT_MOC_LITERAL(18, 421, 24), // "mf_SlotCapturePointCloud"
QT_MOC_LITERAL(19, 446, 14), // "degreesRotated"
QT_MOC_LITERAL(20, 461, 35), // "mf_SlotCapturePointCloudButto..."
QT_MOC_LITERAL(21, 497, 22), // "mf_SlotUpdateStatusBar"
QT_MOC_LITERAL(22, 520, 6), // "status"
QT_MOC_LITERAL(23, 527, 11) // "statusColor"

    },
    "TDK_ScanWindow\0mf_SignalStatusChanged\0"
    "\0mf_SignalDatabasePointCloudUpdated\0"
    "mf_SignalDatabaseRegisteredPointCloudUpdated\0"
    "mf_SignalNumberOfPointCloudUpdated\0"
    "mf_SlotUpdateWindow\0sensorIndex\0"
    "mf_SlotUpdateBoundingBox\0"
    "mf_SlotActivateFiltering\0flagFiltering\0"
    "mf_SlotPointCloudRegistration\0"
    "flagRealTimeScan\0mf_SlotStartScan\0"
    "mf_SlotStopScan\0mf_SlotHandlePlatformParameters\0"
    "flagEnablePlatformParameters\0"
    "mf_SlotUpdatePointCloudStream\0"
    "mf_SlotCapturePointCloud\0degreesRotated\0"
    "mf_SlotCapturePointCloudButtonClick\0"
    "mf_SlotUpdateStatusBar\0status\0statusColor"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_TDK_ScanWindow[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      15,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    2,   89,    2, 0x06 /* Public */,
       3,    0,   94,    2, 0x06 /* Public */,
       4,    0,   95,    2, 0x06 /* Public */,
       5,    1,   96,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       6,    1,   99,    2, 0x0a /* Public */,
       8,    0,  102,    2, 0x0a /* Public */,
       9,    1,  103,    2, 0x0a /* Public */,
      11,    1,  106,    2, 0x0a /* Public */,
      13,    0,  109,    2, 0x0a /* Public */,
      14,    0,  110,    2, 0x0a /* Public */,
      15,    1,  111,    2, 0x0a /* Public */,
      17,    0,  114,    2, 0x0a /* Public */,
      18,    1,  115,    2, 0x0a /* Public */,
      20,    0,  118,    2, 0x0a /* Public */,
      21,    2,  119,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString, QMetaType::QColor,    2,    2,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    2,

 // slots: parameters
    QMetaType::Void, QMetaType::Int,    7,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,   10,
    QMetaType::Void, QMetaType::Bool,   12,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,   16,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   19,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString, QMetaType::QColor,   22,   23,

       0        // eod
};

void TDK_ScanWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        TDK_ScanWindow *_t = static_cast<TDK_ScanWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->mf_SignalStatusChanged((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QColor(*)>(_a[2]))); break;
        case 1: _t->mf_SignalDatabasePointCloudUpdated(); break;
        case 2: _t->mf_SignalDatabaseRegisteredPointCloudUpdated(); break;
        case 3: _t->mf_SignalNumberOfPointCloudUpdated((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->mf_SlotUpdateWindow((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 5: _t->mf_SlotUpdateBoundingBox(); break;
        case 6: _t->mf_SlotActivateFiltering((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 7: _t->mf_SlotPointCloudRegistration((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 8: _t->mf_SlotStartScan(); break;
        case 9: _t->mf_SlotStopScan(); break;
        case 10: _t->mf_SlotHandlePlatformParameters((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 11: _t->mf_SlotUpdatePointCloudStream(); break;
        case 12: _t->mf_SlotCapturePointCloud((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 13: _t->mf_SlotCapturePointCloudButtonClick(); break;
        case 14: _t->mf_SlotUpdateStatusBar((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QColor(*)>(_a[2]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (TDK_ScanWindow::*_t)(QString , QColor );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&TDK_ScanWindow::mf_SignalStatusChanged)) {
                *result = 0;
                return;
            }
        }
        {
            typedef void (TDK_ScanWindow::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&TDK_ScanWindow::mf_SignalDatabasePointCloudUpdated)) {
                *result = 1;
                return;
            }
        }
        {
            typedef void (TDK_ScanWindow::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&TDK_ScanWindow::mf_SignalDatabaseRegisteredPointCloudUpdated)) {
                *result = 2;
                return;
            }
        }
        {
            typedef void (TDK_ScanWindow::*_t)(int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&TDK_ScanWindow::mf_SignalNumberOfPointCloudUpdated)) {
                *result = 3;
                return;
            }
        }
    }
}

const QMetaObject TDK_ScanWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_TDK_ScanWindow.data,
      qt_meta_data_TDK_ScanWindow,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *TDK_ScanWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *TDK_ScanWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_TDK_ScanWindow.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int TDK_ScanWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 15)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 15;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 15)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 15;
    }
    return _id;
}

// SIGNAL 0
void TDK_ScanWindow::mf_SignalStatusChanged(QString _t1, QColor _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void TDK_ScanWindow::mf_SignalDatabasePointCloudUpdated()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}

// SIGNAL 2
void TDK_ScanWindow::mf_SignalDatabaseRegisteredPointCloudUpdated()
{
    QMetaObject::activate(this, &staticMetaObject, 2, nullptr);
}

// SIGNAL 3
void TDK_ScanWindow::mf_SignalNumberOfPointCloudUpdated(int _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
