#pragma once

#include <QObject>
#include "httpserver/httpresponse.h"

#include "httpserver/httprequest.h"
#include "qwebappsocketupgrade_global.h"
#include <qnetworkaccessmanager.h>
#include <qnetworkrequest.h>
#include <qnetworkreply.h>
#include <qthread.h>

namespace qtWebApp
{

	class  QWEBAPPSOCKETUPGRADE_EXPORT HttpClientRequest
	{

	public:

		HttpClientRequest();
		HttpClientRequest(QNetworkAccessManager::Operation operation) :operation(operation) {};
		HttpClientRequest(QNetworkAccessManager::Operation operation, QString url) :operation(operation), url(url) {};
		QUrl url;
		QNetworkAccessManager::Operation operation;
		QByteArray body;
		uint offset;
		QString path;
		QMap<QByteArray, QByteArray> headers;
		void setParams(QString key, QString value) { params.insert(key, value); };
		void setHeader(QString Key, QString value) { this->headers.insert(Key.toUtf8(), value.toUtf8()); };
		void setBody(QByteArray body) { this->body.append(body); }
		QMap<QByteArray, QByteArray> getHeaders() { return this->headers; };
		QByteArray getBody() { return this->body; }
		QNetworkAccessManager::Operation getMethod() { return operation; };
		QMap<QString, QString> params;
		QString getPath() { return path; };
		void setPath(QString path) { this->path = path; }
		QNetworkRequest getRequest(QString localHost, QString port);
	};
	class  QWEBAPPSOCKETUPGRADE_EXPORT HttpClientResponse
	{

	public:

		HttpClientResponse(int status) { this->statusCode = status; };
		HttpClientResponse(QNetworkReply* reply)
		{
			body = reply->readAll();
			url = reply->url();
			reply->errorString();
            for (QNetworkReply::RawHeaderPair header : reply->rawHeaderPairs())
			{
				headers.insert(header.first, header.second);
			}
			statusCode = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();
			if (statusCode != 200)
			{
				errMsg = reply->errorString();
				if (errMsg.contains("Server replied", Qt::CaseInsensitive))
				{
				
					int i = errMsg.lastIndexOf(QString(":"));
					errMsg.remove(0,i+1);
				}
					
			}
		};
		~HttpClientResponse()
		{}
		QUrl url;
		int statusCode = 400;
		QByteArray body;
		uint offset;
		QString errMsg;
		QMap<QString, QString> headers;
		QString getHeader(QString Key) { return this->headers.value(Key); };
		QByteArray getBody() { return body; }
		bool isSuccellfull() { return (statusCode == 200); };
		int getStatusCode() { return statusCode; };
		QString getErrorMsg() { return errMsg; };

	};

	class QWEBAPPSOCKETUPGRADE_EXPORT  HttpClient:public QThread
	{

		
	public:

		HttpClient(QString address, quint16 port);
		~HttpClient()
		{
			//delete m_manager;
		};


		HttpClientResponse* send(HttpClientRequest* request);
	private:
		QString localHost;
		quint16 port;
		QNetworkAccessManager* m_manager;

	};
}
