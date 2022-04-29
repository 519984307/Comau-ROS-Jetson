CONFIG += staticlib
TARGET = QtWebSockets
TEMPLATE = lib
QT = core-private network

PUBLIC_HEADERS += \
    $$PWD/qwebsockets_global.h \
    $$PWD/qwebsocket.h \
    $$PWD/qwebsocketserver.h \
    $$PWD/qwebsocketprotocol.h \
    $$PWD/qwebsocketcorsauthenticator.h \
    $$PWD/qmaskgenerator.h

PRIVATE_HEADERS += \
    $$PWD/qwebsocket_p.h \
    $$PWD/qwebsocketserver_p.h \
    $$PWD/qwebsocketprotocol_p.h \
    $$PWD/qwebsockethandshakerequest_p.h \
    $$PWD/qwebsockethandshakeresponse_p.h \
    $$PWD/qwebsocketdataprocessor_p.h \
    $$PWD/qwebsocketcorsauthenticator_p.h \
    $$PWD/qwebsocketframe_p.h \
    $$PWD/qdefaultmaskgenerator_p.h

SOURCES += \
    $$PWD/qwebsocket.cpp \
    $$PWD/qwebsocket_p.cpp \
    $$PWD/qwebsocketserver.cpp \
    $$PWD/qwebsocketserver_p.cpp \
    $$PWD/qwebsocketprotocol.cpp \
    $$PWD/qwebsockethandshakerequest.cpp \
    $$PWD/qwebsockethandshakeresponse.cpp \
    $$PWD/qwebsocketdataprocessor.cpp \
    $$PWD/qwebsocketcorsauthenticator.cpp \
    $$PWD/qwebsocketframe.cpp \
    $$PWD/qmaskgenerator.cpp \
    $$PWD/qdefaultmaskgenerator_p.cpp

 
    SOURCES += $$PWD/qsslserver.cpp
    PRIVATE_HEADERS += $$PWD/qsslserver_p.h
 

HEADERS += $$PUBLIC_HEADERS $$PRIVATE_HEADERS

#load(qt_module)
