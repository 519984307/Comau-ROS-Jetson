#include "RosActionService.h"
#include "DbManagementExceptions.h"

RosActionService::RosActionService(QSettings* settings, ROSNode *rn) : rn(rn)
{
    // Read some settings from .ini
    this->settings = settings;
    QString dbPath = settings->value("dbPath", "").toString();
    QString host = settings->value("host", "").toString();
    quint16 port = settings->value("port", "").toUInt();
    serviceHost = settings->value("serviceHost", "").toString();
    servicePort = settings->value("servicePort", "").toUInt();
    url = settings->value("host").toString() + ":" + settings->value("port").toString();
    // Connection to local database
    this->mdbConnection = new DatabaseConnectionSqlite(dbPath);
    try
    {
        QString err;
        mdbConnection->openDatabase(&err);
    }
    catch (Exceptions::DbConnectionFailedException& ex){};

    if (mdbConnection->isOpen())
    {
        try
        {
            actionDAL = new RosActionDAL(mdbConnection);
            actionList = actionDAL->getAll();
            for (int i = 0; i < actionList.size(); i++)
            {
                Action* action = actionList.at(i);
                actionMap.insert(action->getActionName(), action);
            }
        }
        catch (Exceptions::DbQueryNotExecutedException& ex)
        {
        }
        catch (Exceptions::DbConnectionFailedException) {}
        catch (Exceptions::DbQuerySelectMoreThanOneRecordException) {}
        catch (Exceptions::DbRecordNotFoundException) {}
        catch (Exceptions::DbRecordNotFoundExceptionNoParam) {}
        catch (Exceptions::PrimaryKeyViolation) {}
        catch (Exceptions::ForeignKeyViolation) {}
    }
}

void RosActionService::serviceRequest(HttpRequest& request, HttpResponse& response)
{
    /*Se non sei nei casi di action = Run / Update / Create verrà chiamata questa funzione.Lo
        sviluppatore può implementarla in base a ciò che gli serve*/
    QByteArray action = request.getParameter("action"); // i.e. create,run,update - mandatory
    QByteArray actionName = request.getParameter("actionName"); // i.e "Test" in body from postman
    
    //Uncomment if action different from [run,create,update] are used
    /*switch (httCustompParameter::fromStringToEnum(action))
    {

    case httCustompParameter::CustomAction1:

        break;
    case httCustompParameter::CustomAction2:

        break;
    }*/
}

QJsonDocument RosActionService::run(QString actionName, QJsonDocument resultOfPreviousAction)
{
    return actionMap.value(actionName)->run(resultOfPreviousAction);
}

bool RosActionService::create(QJsonDocument actionParams)
{
    RosAction* newAction = new RosAction(rn);
    try
    {

        bool created = newAction->create(actionParams);
        if (created)
        {
            actionMap.insert(newAction->getActionName(), newAction);
            //actionDAL->add(newAction);
        }       
        else
        {
            delete newAction;
        }
        return created;
    }
    catch (ActionException& ex)
    {
        delete newAction;
        throw ActionException(HttpResponseStatusCode::InternalServerError, ex.getErrorMsg());
    }
}

bool RosActionService::update(QString actionName, QJsonDocument actionParams)
{
    RosAction* action = (RosAction*)actionMap.value(actionName);

    try
    {

        bool created = action->create(actionParams);
        if (created)
        {
            actionMap.remove(actionName);
            actionMap.insert(action->getActionName(), action);
        }

        return created;
    }
    catch (ActionException& ex)
    {
        throw ActionException(HttpResponseStatusCode::InternalServerError, ex.getErrorMsg());
    }

}

void RosActionService::websocketbinaryFrameReceived(QWebSocket* ws, const QByteArray& data, bool final)
{
    qCritical("HttpRequestHandler: you need to override the websocketbinaryFrameReceived function");
    qDebug("HttpRequestHandler: Default websocket echo implementation. You need to override the websocketbinaryFrameReceived function");

    ws->sendBinaryMessage(data);
}

void RosActionService::websocketTextMessage(QWebSocket* ws, const QString& data)
{
    ws->sendTextMessage("Key not recognized");
}



