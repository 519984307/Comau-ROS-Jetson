/****************************************************************************
** Meta object code from reading C++ file 'httplistener.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.5)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../qwebappsocketupgrade/httpserver/httplistener.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'httplistener.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.5. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_qtWebApp__HttpListener_t {
    QByteArrayData data[5];
    char stringdata0[76];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_qtWebApp__HttpListener_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_qtWebApp__HttpListener_t qt_meta_stringdata_qtWebApp__HttpListener = {
    {
QT_MOC_LITERAL(0, 0, 22), // "qtWebApp::HttpListener"
QT_MOC_LITERAL(1, 23, 16), // "handleConnection"
QT_MOC_LITERAL(2, 40, 0), // ""
QT_MOC_LITERAL(3, 41, 17), // "tSocketDescriptor"
QT_MOC_LITERAL(4, 59, 16) // "socketDescriptor"

    },
    "qtWebApp::HttpListener\0handleConnection\0"
    "\0tSocketDescriptor\0socketDescriptor"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_qtWebApp__HttpListener[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       1,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   19,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,

       0        // eod
};

void qtWebApp::HttpListener::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        HttpListener *_t = static_cast<HttpListener *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->handleConnection((*reinterpret_cast< tSocketDescriptor(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            typedef void (HttpListener::*_t)(tSocketDescriptor );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&HttpListener::handleConnection)) {
                *result = 0;
                return;
            }
        }
    }
}

const QMetaObject qtWebApp::HttpListener::staticMetaObject = {
    { &QTcpServer::staticMetaObject, qt_meta_stringdata_qtWebApp__HttpListener.data,
      qt_meta_data_qtWebApp__HttpListener,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *qtWebApp::HttpListener::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *qtWebApp::HttpListener::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_qtWebApp__HttpListener.stringdata0))
        return static_cast<void*>(this);
    return QTcpServer::qt_metacast(_clname);
}

int qtWebApp::HttpListener::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QTcpServer::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 1)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 1;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 1)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 1;
    }
    return _id;
}

// SIGNAL 0
void qtWebApp::HttpListener::handleConnection(tSocketDescriptor _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
