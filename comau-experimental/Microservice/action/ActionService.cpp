#include "ActionService.h"
#include "qjsonobject.h"
#include "ActionType.h"


void ActionService::service(HttpRequest& request, HttpResponse& response)
{
	QString command = request.getParameter("action");
	QJsonDocument body;
	QJsonParseError err;
	try
	{
		switch (httpParameter::fromStringToEnum(command))
		{
		case httpParameter::Run:
			if (!request.getParameter("actionName").isEmpty())
			{
				QByteArray input = request.getBody();
				QJsonParseError err;
				QJsonDocument inputDoc = QJsonDocument::fromJson(input, &err);
				if (err.error != QJsonParseError::NoError)
					throw ActionException(HttpResponseStatusCode::BadRequest, "error in json");
				try
				{
					QJsonDocument responseDoc = run(request.getParameter("actionName"),inputDoc);
					response.setHeader("content-type", "application/json");
					response.write(responseDoc.toJson());
				}
				catch (ActionException& ex)
				{
					response.setStatus(ex.getStatusCode(), ex.getErrorMsg().toUtf8());
				}
			}
			else
				response.setStatus(HttpResponseStatusCode::BadRequest, "No action name");
			break;
		case httpParameter::Create:
	
			
				body = QJsonDocument::fromJson(request.getBody(), &err);
				if (err.error != QJsonParseError::NoError)
					throw ActionException(HttpResponseStatusCode::BadRequest, "error in json");
				try
				{
					bool created=create(body);
					if (created)
					{
						response.setStatus(200, "created");
					}

					else
						response.setStatus(HttpResponseStatusCode::InternalServerError, "Not created");
				}
				catch (ActionException& ex)
				{
					response.setStatus(ex.getStatusCode(), ex.getErrorMsg().toUtf8());
				}
			break;
		case httpParameter::Update:
			if (request.getParameter("actionName").isEmpty())
				throw ActionException(HttpResponseStatusCode::BadRequest, "No action name");
			
			body = QJsonDocument::fromJson(request.getBody(), &err);
			if (err.error == QJsonParseError::NoError)
			{
				
				bool executed = update(request.getParameter("actionName"),body);

				if (executed)
					response.setStatus(200);
				else
					response.setStatus(HttpResponseStatusCode::InternalServerError, "error during first configuration");
			}
			else
				response.setStatus(HttpResponseStatusCode::InternalServerError, err.errorString().toUtf8());
			break;
		case httpParameter::UpdateTypeInCentralDb:
			try
			{
				ActionType type(getSettings());
				HttpClient* serviceClient = new HttpClient(getActionServiceHost(), getActionServicePort());
				HttpClientRequest* requestClient = new HttpClientRequest(CommandRequest::fromCustomCommandToQtCommand(CommandRequest::Post));
				requestClient->setParams("action", "updateType");
				requestClient->setParams("actionType", type.getPreviousName());
				requestClient->setHeader("content-type", "application/json");
				requestClient->setBody(parseJsonOfType().toJson());
				HttpClientResponse* reply = serviceClient->send(requestClient);
				int statusCode = reply->statusCode;
				if (statusCode != 200)
				{

					throw (ActionException(reply->errMsg, statusCode));
				}
				delete requestClient;
				delete reply;
				delete serviceClient;

			}
			catch (ActionException& ex)
			{
				response.setStatus(ex.getStatusCode(), ex.getErrorMsg().toUtf8());
			}
			break;
		case httpParameter::AddTypeInCentralDb:
			try
			{

				
				HttpClientRequest* requestClient = new HttpClientRequest(CommandRequest::fromCustomCommandToQtCommand(CommandRequest::Post));
				requestClient->setParams("action", "addType");
				requestClient->setHeader("content-type", "application/json");
				requestClient->setBody(parseJsonOfType().toJson());
				HttpClient* serviceClient = new HttpClient(getActionServiceHost(), getActionServicePort());
				HttpClientResponse* reply = serviceClient->send(requestClient);
				int statusCode = reply->statusCode;
				if (statusCode != 200)
				{

					throw (ActionException(reply->errMsg, statusCode));
				}
				delete requestClient;
				delete reply;
				delete serviceClient;

			}
			catch (ActionException& ex)
			{
				response.setStatus(ex.getStatusCode(), ex.getErrorMsg().toUtf8());
			}
			break;
		
		default:
			serviceRequest(request, response);
			break;
		}
	}
	catch (ActionException& ex)
	{
		response.setStatus(ex.getStatusCode(), ex.getErrorMsg().toUtf8());
	}

}
QJsonDocument ActionService::parseJsonOfType()
{

	QMap<QString, int> inputStruct = getDefaultPointerOfAction()->getInputStruct();
	QJsonObject inputObj;
	QMapIterator<QString, int> iterator(inputStruct);
	while (iterator.hasNext())
	{
		iterator.next();
		inputObj.insert(iterator.key(), iterator.value());
	}
	QMap<QString, int> outputStruct = getDefaultPointerOfAction()->getOutputStruct();
	QJsonObject outputObj;
	QMapIterator<QString, int> outIterator(outputStruct);
	while (outIterator.hasNext())
	{
		outIterator.next();
		outputObj.insert(outIterator.key(), outIterator.value());
	}
	QJsonObject obj;
	ActionType type(getSettings());
	obj.insert("input", inputObj);
	obj.insert("output", outputObj);
	obj.insert("actionType", type.getName());
	obj.insert("url", getUrl());
	obj.insert("enabled", type.isEnabled());
	obj.insert("icon", QLatin1String(type.addIcon().toBase64()));
	obj.insert("color", type.addColor());
	QJsonDocument typeDoc;
	typeDoc.setObject(obj);
	return typeDoc;
}