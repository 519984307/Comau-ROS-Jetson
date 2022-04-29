QT -= gui
QT += core network sql widgets core-private
TEMPLATE = lib
DEFINES += QWEBAPPSOCKETUPGRADE_LIBRARY

CONFIG += c++11

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    QWebAppSocketUpgrade.cpp \
    httpClient.cpp \
    httpserver/httpconnectionhandler.cpp \
    httpserver/httpconnectionhandlerpool.cpp \
    httpserver/httpcookie.cpp \
    httpserver/httpglobal.cpp \
    httpserver/httplistener.cpp \
    httpserver/httprequest.cpp \
    httpserver/httprequesthandler.cpp \
    httpserver/httpresponse.cpp \
    httpserver/httpsession.cpp \
    httpserver/httpsessionstore.cpp \
    httpserver/staticfilecontroller.cpp \
    qtwebsockets/src/websockets/qdefaultmaskgenerator_p.cpp \
    qtwebsockets/src/websockets/qmaskgenerator.cpp \
    qtwebsockets/src/websockets/qsslserver.cpp \
    qtwebsockets/src/websockets/qwebsocket.cpp \
    qtwebsockets/src/websockets/qwebsocket_p.cpp \
    qtwebsockets/src/websockets/qwebsocketcorsauthenticator.cpp \
    qtwebsockets/src/websockets/qwebsocketdataprocessor.cpp \
    qtwebsockets/src/websockets/qwebsocketframe.cpp \
    qtwebsockets/src/websockets/qwebsockethandshakerequest.cpp \
    qtwebsockets/src/websockets/qwebsockethandshakeresponse.cpp \
    qtwebsockets/src/websockets/qwebsocketprotocol.cpp \
    qtwebsockets/src/websockets/qwebsocketserver.cpp \
    qtwebsockets/src/websockets/qwebsocketserver_p.cpp

HEADERS += \
    HttpResponseStatusCode.h \
    httpClient.h \
    httpserver/httpconnectionhandler.h \
    httpserver/httpconnectionhandlerpool.h \
    httpserver/httpcookie.h \
    httpserver/httpglobal.h \
    httpserver/httplistener.h \
    httpserver/httprequest.h \
    httpserver/httprequesthandler.h \
    httpserver/httpresponse.h \
    httpserver/httpsession.h \
    httpserver/httpsessionstore.h \
    httpserver/staticfilecontroller.h \
    qtwebsockets/src/websockets/qdefaultmaskgenerator_p.h \
    qtwebsockets/src/websockets/qmaskgenerator.h \
    qtwebsockets/src/websockets/qsslserver_p.h \
    qtwebsockets/src/websockets/qwebsocket.h \
    qtwebsockets/src/websockets/qwebsocket_p.h \
    qtwebsockets/src/websockets/qwebsocketcorsauthenticator.h \
    qtwebsockets/src/websockets/qwebsocketcorsauthenticator_p.h \
    qtwebsockets/src/websockets/qwebsocketdataprocessor_p.h \
    qtwebsockets/src/websockets/qwebsocketframe_p.h \
    qtwebsockets/src/websockets/qwebsockethandshakerequest_p.h \
    qtwebsockets/src/websockets/qwebsockethandshakeresponse_p.h \
    qtwebsockets/src/websockets/qwebsocketprotocol.h \
    qtwebsockets/src/websockets/qwebsocketprotocol_p.h \
    qtwebsockets/src/websockets/qwebsockets_global.h \
    qtwebsockets/src/websockets/qwebsocketserver.h \
    qtwebsockets/src/websockets/qwebsocketserver_p.h \
    qwebappsocketupgrade_global.h \
    QWebAppSocketUpgrade.h

# Default rules for deployment.
unix {
    target.path = /usr/lib
}
!isEmpty(target.path): INSTALLS += target

SUBDIRS += \
    qtwebsockets/qtwebsockets.pro \
    qtwebsockets/src/src.pro \
    qtwebsockets/src/websockets/websockets.pro

DISTFILES += \
    httpserver/httpserver.pri \
    qtwebsockets/LGPL_EXCEPTION.txt \
    qtwebsockets/LICENSE.GPL2 \
    qtwebsockets/LICENSE.GPL3 \
    qtwebsockets/LICENSE.GPL3-EXCEPT \
    qtwebsockets/LICENSE.GPLv3 \
    qtwebsockets/LICENSE.LGPL3 \
    qtwebsockets/LICENSE.LGPLv21 \
    qtwebsockets/LICENSE.LGPLv3 \
    qtwebsockets/README.md \
    qtwebsockets/src/websockets/qtwebsockets.pri \
    qtwebsockets/sync.profile
