/****************************************************************************
** Meta object code from reading C++ file 'qsslserver_p.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.5)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../qwebappsocketupgrade/qtwebsockets/src/websockets/qsslserver_p.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#include <QtCore/QList>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'qsslserver_p.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.5. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_QSslServer_t {
    QByteArrayData data[12];
    char stringdata0[181];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_QSslServer_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_QSslServer_t qt_meta_stringdata_QSslServer = {
    {
QT_MOC_LITERAL(0, 0, 10), // "QSslServer"
QT_MOC_LITERAL(1, 11, 9), // "sslErrors"
QT_MOC_LITERAL(2, 21, 0), // ""
QT_MOC_LITERAL(3, 22, 16), // "QList<QSslError>"
QT_MOC_LITERAL(4, 39, 6), // "errors"
QT_MOC_LITERAL(5, 46, 15), // "peerVerifyError"
QT_MOC_LITERAL(6, 62, 9), // "QSslError"
QT_MOC_LITERAL(7, 72, 5), // "error"
QT_MOC_LITERAL(8, 78, 22), // "newEncryptedConnection"
QT_MOC_LITERAL(9, 101, 34), // "preSharedKeyAuthenticationReq..."
QT_MOC_LITERAL(10, 136, 30), // "QSslPreSharedKeyAuthenticator*"
QT_MOC_LITERAL(11, 167, 13) // "authenticator"

    },
    "QSslServer\0sslErrors\0\0QList<QSslError>\0"
    "errors\0peerVerifyError\0QSslError\0error\0"
    "newEncryptedConnection\0"
    "preSharedKeyAuthenticationRequired\0"
    "QSslPreSharedKeyAuthenticator*\0"
    "authenticator"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_QSslServer[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   34,    2, 0x06 /* Public */,
       5,    1,   37,    2, 0x06 /* Public */,
       8,    0,   40,    2, 0x06 /* Public */,
       9,    1,   41,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 6,    7,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 10,   11,

       0        // eod
};

void QSslServer::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        QSslServer *_t = static_cast<QSslServer *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->sslErrors((*reinterpret_cast< const QList<QSslError>(*)>(_a[1]))); break;
        case 1: _t->peerVerifyError((*reinterpret_cast< const QSslError(*)>(_a[1]))); break;
        case 2: _t->newEncryptedConnection(); break;
        case 3: _t->preSharedKeyAuthenticationRequired((*reinterpret_cast< QSslPreSharedKeyAuthenticator*(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<int*>(_a[0]) = -1; break;
        case 0:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QList<QSslError> >(); break;
            }
            break;
        case 3:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QSslPreSharedKeyAuthenticator* >(); break;
            }
            break;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            typedef void (QSslServer::*_t)(const QList<QSslError> & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QSslServer::sslErrors)) {
                *result = 0;
                return;
            }
        }
        {
            typedef void (QSslServer::*_t)(const QSslError & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QSslServer::peerVerifyError)) {
                *result = 1;
                return;
            }
        }
        {
            typedef void (QSslServer::*_t)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QSslServer::newEncryptedConnection)) {
                *result = 2;
                return;
            }
        }
        {
            typedef void (QSslServer::*_t)(QSslPreSharedKeyAuthenticator * );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QSslServer::preSharedKeyAuthenticationRequired)) {
                *result = 3;
                return;
            }
        }
    }
}

const QMetaObject QSslServer::staticMetaObject = {
    { &QTcpServer::staticMetaObject, qt_meta_stringdata_QSslServer.data,
      qt_meta_data_QSslServer,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *QSslServer::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *QSslServer::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_QSslServer.stringdata0))
        return static_cast<void*>(this);
    return QTcpServer::qt_metacast(_clname);
}

int QSslServer::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QTcpServer::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    }
    return _id;
}

// SIGNAL 0
void QSslServer::sslErrors(const QList<QSslError> & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void QSslServer::peerVerifyError(const QSslError & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void QSslServer::newEncryptedConnection()
{
    QMetaObject::activate(this, &staticMetaObject, 2, nullptr);
}

// SIGNAL 3
void QSslServer::preSharedKeyAuthenticationRequired(QSslPreSharedKeyAuthenticator * _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
