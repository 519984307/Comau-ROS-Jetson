/****************************************************************************
** Meta object code from reading C++ file 'DbCheckthread.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.5)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../dbmanager/DbCheckthread.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'DbCheckthread.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.5. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_DBCheckThread_t {
    QByteArrayData data[8];
    char stringdata0[81];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_DBCheckThread_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_DBCheckThread_t qt_meta_stringdata_DBCheckThread = {
    {
QT_MOC_LITERAL(0, 0, 13), // "DBCheckThread"
QT_MOC_LITERAL(1, 14, 9), // "setDBFlag"
QT_MOC_LITERAL(2, 24, 0), // ""
QT_MOC_LITERAL(3, 25, 4), // "flag"
QT_MOC_LITERAL(4, 30, 10), // "processEnd"
QT_MOC_LITERAL(5, 41, 10), // "stopSignal"
QT_MOC_LITERAL(6, 52, 11), // "start_timer"
QT_MOC_LITERAL(7, 64, 16) // "slot_timer_start"

    },
    "DBCheckThread\0setDBFlag\0\0flag\0processEnd\0"
    "stopSignal\0start_timer\0slot_timer_start"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_DBCheckThread[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   39,    2, 0x06 /* Public */,
       4,    0,   42,    2, 0x06 /* Public */,
       5,    0,   43,    2, 0x06 /* Public */,
       6,    0,   44,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       7,    0,   45,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void,

       0        // eod
};

void DBCheckThread::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        DBCheckThread *_t = static_cast<DBCheckThread *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->setDBFlag((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 1: _t->processEnd(); break;
        case 2: _t->stopSignal(); break;
        case 3: _t->start_timer(); break;
        case 4: _t->slot_timer_start(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            typedef void (DBCheckThread::*_t)(bool );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&DBCheckThread::setDBFlag)) {
                *result = 0;
                return;
            }
        }
        {
            typedef void (DBCheckThread::*_t)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&DBCheckThread::processEnd)) {
                *result = 1;
                return;
            }
        }
        {
            typedef void (DBCheckThread::*_t)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&DBCheckThread::stopSignal)) {
                *result = 2;
                return;
            }
        }
        {
            typedef void (DBCheckThread::*_t)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&DBCheckThread::start_timer)) {
                *result = 3;
                return;
            }
        }
    }
}

const QMetaObject DBCheckThread::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_DBCheckThread.data,
      qt_meta_data_DBCheckThread,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *DBCheckThread::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *DBCheckThread::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_DBCheckThread.stringdata0))
        return static_cast<void*>(this);
    return QThread::qt_metacast(_clname);
}

int DBCheckThread::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 5)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 5;
    }
    return _id;
}

// SIGNAL 0
void DBCheckThread::setDBFlag(bool _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void DBCheckThread::processEnd()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}

// SIGNAL 2
void DBCheckThread::stopSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 2, nullptr);
}

// SIGNAL 3
void DBCheckThread::start_timer()
{
    QMetaObject::activate(this, &staticMetaObject, 3, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
