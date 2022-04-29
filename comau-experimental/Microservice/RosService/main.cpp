#include <QtCore/QCoreApplication>
#include <QtCore/QCoreApplication>
#include <QSettings>
//#include "../QWebAppSocketUpgrade/httpserver/httplistener.cpp"
#include <httpserver/httplistener.h>
#include "httpserver/httplistener.h"
#include "RosActionService.h"
#include <QtWidgets/QApplication>
//#include <httplistener.cpp>
//#include <qmainwindow.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <ROSPlan.h>
#include "rosnode.h"


int main(int argc, char* argv[])
{

    // MICROSERVICE
    QApplication a(argc, argv);
    QSettings* sett = new QSettings("./RosActionSettings.ini", QSettings::IniFormat);
    ROSNode* rn = new ROSNode(argc,argv,sett);
    HttpListener* httpListener = new HttpListener(sett, new RosActionService(sett,rn), nullptr);

    httpListener->listen();

    int res;
    try {
        res = a.exec();

    }
    catch (const std::bad_alloc&) {

        // clean up here, e.g. save the session
        // and close all config files.
        return 0; // exit the application
    }
    a.exit();

    return res;
    return a.exec();

}
