/****************************************************************************
** Meta object code from reading C++ file 'pclviewer.h'
**
** Created: Sun Oct 30 15:56:06 2016
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "pclviewer.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'pclviewer.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_PCLViewer[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      11,   10,   10,   10, 0x0a,
      33,   10,   10,   10, 0x0a,
      40,   10,   10,   10, 0x0a,
      66,   60,   10,   10, 0x0a,
      91,   60,   10,   10, 0x0a,
     118,   60,   10,   10, 0x0a,
     147,   60,   10,   10, 0x0a,
     175,   10,   10,   10, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_PCLViewer[] = {
    "PCLViewer\0\0randomButtonPressed()\0"
    "load()\0RGBsliderReleased()\0value\0"
    "pSliderValueChanged(int)\0"
    "redSliderValueChanged(int)\0"
    "greenSliderValueChanged(int)\0"
    "blueSliderValueChanged(int)\0"
    "on_load_clicked()\0"
};

void PCLViewer::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        PCLViewer *_t = static_cast<PCLViewer *>(_o);
        switch (_id) {
        case 0: _t->randomButtonPressed(); break;
        case 1: _t->load(); break;
        case 2: _t->RGBsliderReleased(); break;
        case 3: _t->pSliderValueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->redSliderValueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 5: _t->greenSliderValueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 6: _t->blueSliderValueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 7: _t->on_load_clicked(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData PCLViewer::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject PCLViewer::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_PCLViewer,
      qt_meta_data_PCLViewer, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &PCLViewer::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *PCLViewer::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *PCLViewer::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_PCLViewer))
        return static_cast<void*>(const_cast< PCLViewer*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int PCLViewer::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
