#include <httpserver/httprequesthandler.h>
#include <httpserver/httpresponse.h>
#include <httpserver/httprequest.h>
#include <DatabaseConnection.h>
#include "Action.h"
#include "DAL/ActionDAL.h"
#pragma once
using namespace qtWebApp;
class ACTION_EXPORT httpParameter
{
public:
    enum parameter
	{
		Run,
		Create,
		Update,
		AddTypeInCentralDb,
		UpdateTypeInCentralDb,
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
		case AddTypeInCentralDb:
			return "AddType";
		case UpdateTypeInCentralDb:
			return "UpdateType";
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
		if (param.contains("UpdateType", Qt::CaseInsensitive))
			return UpdateTypeInCentralDb;
		if (param.contains("AddType", Qt::CaseInsensitive))
			return AddTypeInCentralDb;
		if (param.contains("Update", Qt::CaseInsensitive))
			return Update;
		return Default;
	};
};
class ACTION_EXPORT ActionService : public qtWebApp::HttpRequestHandler
{
public:
	
	ActionService() {};
	~ActionService() {};
	/*!
	* Chiamata quando il server riceve una richiesta. Prendendo request si possono leggere i parametri (request.getParams(“nome params”)) o il body (request.getBody).
Se nei parametri della request è presente:
action=Run chiamerà la funzione virtuale run della stessa classe (che ogni sviluppatore dovrà sviluppare) passando il Json con gli input e restituirà un json con i risultati.
Action=create: chiamerà la funzione virtuale create della stessa classe (che ogni sviluppatore dovrà sviluppare) passando il Json con gli input. 
Action=Update chiamerà la funzione virtuale update della stessa classe (che ogni sviluppatore dovrà sviluppare) passando il Json con gli input.

	*/
	void service(HttpRequest& request, HttpResponse& response) override;

	/* web socket slot. It is called When a websocket bynary  arrives 
	@params ws: websocket handle 
	@data: data of the request sent by the client*/
	virtual void websocketbinaryFrameReceived(QWebSocket* ws, const QByteArray& data, bool final)=0;
	/* web socket slot. It is called When a websocket tex messages  arrives 
	* 	@params ws: websocket handle 
	@data: data of the request sent by the client*/
	virtual void websocketTextMessage(QWebSocket* ws, const QString& data) =0;
	/* called by the service method. If ypou don't have the request param  action=Run/Update/Create 
	it called this method. The developer could implement it with his action=..
	@param request: request from the client
	@response pointer of the response. Use it to write the messages and to set the status*/
	virtual void serviceRequest(HttpRequest& request, HttpResponse& response) = 0;
	/*! Method that ask the parameters of the action type and parse the json
	*@return json with the action type parametyer
	*/
	QJsonDocument parseJsonOfType();
	/* get a default pointer of the derived class. It is used to call the getInputStruct and getOutputStruct
	@return pointer of the derived class*/
	virtual Action* getDefaultPointerOfAction()=0;

	/* get the ActionService host
	@return the address of the service host*/
	virtual QString getActionServiceHost()=0;
	/* get the ActionService port
	@return the address of the service port*/
	virtual quint16 getActionServicePort() = 0;
	/*
	* get the QSettings pointer
	* @return the pointer of the derived class QSettings
	*/
	virtual QSettings* getSettings() = 0;
	/*
get the url of the current action
@return the url of the derived class
*/
	virtual QString getUrl() = 0;

	/*run the action. 
	@param actionName: action name to run
	@param input. Input Json 
	{"input name"=input value}
	@return json of result {outputName=value}*/
	virtual QJsonDocument run(QString actionName,QJsonDocument input)=0;
	/*
	create the action. 
	@param input: json with the params 	{"name"= value}
	@return flag if it is successfull created
	*/
	virtual bool create(QJsonDocument input)=0;
	/*
update the action.
@param previousActionName: previous name of the action to update
@param input: json with the params 	{"name"= value}
@return flag if it is successfull created
*/
	virtual bool update(QString previousActionName, QJsonDocument input) = 0;

};

