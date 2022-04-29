#include "Action.h"

Action::Action()
{
}
QByteArray Action::sendHttpRequest(HttpClient* httpClient,CommandRequest::commandRequest command, QMap<QString,QString> params,QString path, QByteArray body)
{
	
	HttpClientRequest* requestClient=new HttpClientRequest(CommandRequest::fromCustomCommandToQtCommand(command));
	if(path!="")
		requestClient->setPath(path);
	if (!params.isEmpty())
	{
		QMapIterator<QString, QString> i(params);
		while (i.hasNext())
		{
			i.next();
			requestClient->setParams(i.key(), i.value());
		}
	}
	if (!body.isEmpty())
		requestClient->setBody(body);
	HttpClientResponse* reply = httpClient->send(requestClient);

	int statusCode = reply->statusCode;
	if (statusCode != 200)
	{

		throw (ActionException(reply->errMsg, statusCode));
	}


	QByteArray responseBody = reply->getBody();
	delete requestClient;
	delete reply;
	return responseBody;
	
}