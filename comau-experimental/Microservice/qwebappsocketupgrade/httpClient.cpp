#include "httpClient.h"



#include <QNetworkRequest>
#include <QNetworkReply>
#include <QTimer>
#include <QEventLoop>

using namespace qtWebApp;

HttpClient::HttpClient(QString address, quint16 port){

    m_manager = new QNetworkAccessManager(this);
    this->localHost = address;
    this->port = port;
    m_manager->connectToHost(localHost, port);

}



HttpClientResponse* HttpClient::send(HttpClientRequest* request)
{

    QNetworkRequest networkRequest = request->getRequest(localHost,QString::number(port));
    QString url = networkRequest.url().toString();
    QNetworkReply* networkReply;
 
    switch (request->getMethod())
    {
    case QNetworkAccessManager::GetOperation:
        networkReply = m_manager->get(networkRequest);
        break;
    case QNetworkAccessManager::PostOperation:
      
        networkReply = m_manager->post(networkRequest, request->getBody());
        break;
    case QNetworkAccessManager::Operation::PutOperation:
        networkReply = m_manager->put(networkRequest, request->getBody());
        break;
    case QNetworkAccessManager::Operation::DeleteOperation:
        networkReply = m_manager->deleteResource(networkRequest);
        break;
    case QNetworkAccessManager::Operation::HeadOperation:
        networkReply = m_manager->head(networkRequest);
        break;
    case QNetworkAccessManager::Operation::CustomOperation:
        networkReply = m_manager->sendCustomRequest(networkRequest, request->getBody());
        break;
    default:
        throw ("Unknown or unimplemented HTTP method '" + request->getMethod());
        break;
    }


    QTimer timer;
    QEventLoop loop;

    QObject::connect(&timer, SIGNAL(timeout()), &loop, SLOT(quit()));
    QObject::connect(networkReply, SIGNAL(finished()), &loop, SLOT(quit()));

    timer.setSingleShot(true);
    timer.start(5000); // 5 seconds

    loop.exec();

    HttpClientResponse* response;
    if (timer.isActive()) {
        timer.stop();
        response = new HttpClientResponse(networkReply);
     
    }
    else {
        networkReply->abort();
        response = new HttpClientResponse(500);
    }
   
    
    QObject::disconnect(networkReply, SIGNAL(finished()), &loop, SLOT(quit()));
    delete networkReply;

    return response;
}

QNetworkRequest HttpClientRequest::getRequest(QString localHost,QString port)
{
    QNetworkRequest networkRequest;
    QString url="http://";
 
    url.append(localHost + ":" + port);
    if (path.isEmpty())
        url.append("/");
    else
    {
        if (path.at(0) != "/")
            url.append("/");
        url.append(path);
        if (!path.endsWith("/"))
            url.append("/");


    }
    url.append("?");
    int i = 0;
    foreach(QString paramKey, params.keys()) {
        if (i > 0)
        {
            url.append("&");
        }
        url.append(paramKey + "=" + params.value(paramKey));
        i++;
    }
    networkRequest.setUrl(url);
    
    foreach(QByteArray key, headers.keys()) {
        networkRequest.setRawHeader(
            key,
            headers.value(key)
        );
    }
    networkRequest.setHeader(QNetworkRequest::KnownHeaders::ContentLengthHeader, body.size());
    return networkRequest;
}

HttpClientRequest::HttpClientRequest()
{
    headers.clear();
    headers.insert("Connection", "keep-alive");
    headers.insert("Transfer-Encoding", "chunked");
    headers.insert("Access-Control-Allow-Origin", "*");
    headers.insert("Access-Control-Allow-Methods", "HEAD, GET, POST, PUT, PATCH, DELETE, OPTIONS");
    headers.insert("Access-Control-Allow-Headers", "X-API-KEY, Origin, X-Requested-With, Content-Type, Accept, Access-Control-Request-Method,Access-Control-Request-Headers, Authorization");
}
