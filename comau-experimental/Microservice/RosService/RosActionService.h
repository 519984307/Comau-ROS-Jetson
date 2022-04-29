#pragma once
#include <QObject>
#include "qjsonobject.h"
#include "qjsondocument.h"
#include "httpserver/httprequesthandler.h"
#include "httpserver/httprequesthandler.h"
#include <QtCore/qbuffer.h>
#include <qimage.h>

#include <QtCore/qeventloop.h>
#include <QNetworkReply>
#include <DatabaseConnection.h>
#include <DatabaseConnectionSqlite.h>
#include "DAL/RosActionDAL.h"
#include "RosAction.h"
#include "ActionService.h"
#include "rosnode.h"
using namespace qtWebApp;

class httCustompParameter
{
public:
    enum parameter
	{
		Run,
		Create,
		Update,
		Default,
	};
	const QString fromEnumToString(parameter param)
	{
		switch (param)
		{
		case Run:
			return "Run";

		case Create:
			return "Create";
		case Update:
			return "Update";

		default:
			return "";
		}
	};
	static parameter fromStringToEnum(QString param)
	{
		if (param.contains("Run", Qt::CaseInsensitive))
			return Run;

		if (param.contains("Create", Qt::CaseInsensitive))
			return Create;
		if (param.contains("Update", Qt::CaseInsensitive))
			return Update;
		return Default;
	};
};

class RosActionService :public ActionService
{
public:

    RosActionService(QSettings* settings){};
    RosActionService(QSettings *settings, ROSNode *rn);
    ~RosActionService() {std::cout << "RosService destructor" << std::endl;};


	HttpResponse* responseExt;
	QWebSocket* webSocketLive;
	DatabaseConnection* mdbConnection;
	DAL::ActionDAL* actionDAL;
	QList<Action*> actionList;
	QMap<QString, Action*> actionMap;
	QString serviceHost;
	quint16 servicePort;
	QString url;
	QSettings* settings;
    ROSNode* rn;

	void websocketbinaryFrameReceived(QWebSocket* ws, const QByteArray& data, bool final) override;
	void websocketTextMessage(QWebSocket* ws, const QString& data) override;
	void serviceRequest(HttpRequest& request, HttpResponse& response) override;
	Action* getDefaultPointerOfAction() { return  new RosAction(); };
	QString getActionServiceHost() override { return serviceHost; };
	quint16 getActionServicePort() override { return servicePort; };
	QSettings* getSettings() override { return settings; };
	QString getUrl() override { return url; };
    QJsonDocument run(QString actionName, QJsonDocument resultOfPreviousAction) override;
	bool create(QJsonDocument actionParams) override;
	bool update(QString actionName, QJsonDocument actionParams) override;



};

