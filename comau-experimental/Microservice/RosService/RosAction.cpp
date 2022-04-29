#include "RosAction.h"
#include "DAL/RosActionDAL.h"

RosAction::~RosAction()
{
    std::cout << "RosAction desrtuctor" << std::endl;
}

bool RosAction::create(QJsonDocument inputParameter)
{
    
    QJsonDocument doc = inputParameter;

    name = doc.object().value("actionName").toString();
    command = commandManager.fromStringToCommand(doc.object().value("command").toString());

    if (name.isEmpty())
    {
        throw(ActionException("Empty name", HttpResponseStatusCode::InternalServerError));
    }

    qDebug() << "Action: " + name + " created.";
    return true;
}

QJsonDocument RosAction::run(QJsonDocument resultOfPreviousAction)
{
    qDebug() << "Action: " + getActionName() + " running...";
    QByteArray responseByte;
    QJsonDocument doc = resultOfPreviousAction;
    QJsonDocument docret;
    QByteArray response = 0;
    QJsonObject j;
    command = commandManager.fromStringToCommand(doc.object().value("command").toString());

    bool connected = false;
    bool planned = false;
    bool move_opt = false;
    bool executed = false;
    bool init = false;
    bool load_scene = false;
    bool tool = false;
    switch (command)
    {
    case Command::InitFramework:       
        init = rp.initFramework();
        rn->initClient();
        // Response Body    
        j["Request"] = "Framework initialization";
        j["Response"] = init;
        response = QJsonDocument(j).toJson();
        docret = QJsonDocument::fromJson(response);
        return docret;
        break;

    case Command::ConnectToRobot:
        connected = rp.connectToRobot();
        // Response Body
        j["Request"] = "Connect To Robot";
        j["Response"] = connected;
        response = QJsonDocument(j).toJson();
        docret = QJsonDocument::fromJson(response);
        return docret;
        break;

    case Command::InitRealRobot:

        init = rp.initRealRobot();
        rn->initClient();
        // Response Body
        j["Request"] = "Framework initialization";
        j["Response"] = init;
        response = QJsonDocument(j).toJson();
        docret = QJsonDocument::fromJson(response);
        return docret;
        break;

    case Command::LoadScene:
        load_scene = rn->publishPCL(doc);
        // Response Body    
        j["Request"] = "Load scene";
        j["Response"] = load_scene;
        response = QJsonDocument(j).toJson();
        docret = QJsonDocument::fromJson(response);
        return docret;
        break;

    case Command::MoveP2P:  
        planned = rn->sendTargetGoal(doc);
        // Response Body    
        j["Request"] = "MoveP2P";
        j["Response"] = planned;
        response = QJsonDocument(j).toJson();
        docret = QJsonDocument::fromJson(response);
        return docret;      
        break;

    case Command::Test:
        move_opt = rn->sendTargetGoalOpt(doc);
        // Response Body
        j["Request"] = "Test";
        j["Response"] = move_opt;
        response = QJsonDocument(j).toJson();
        docret = QJsonDocument::fromJson(response);
        return docret;
        break;

    case Command::Execute:       
        executed = rn->execute(doc);
        // Response Body
        j["Request"] = "Execute";
        j["Response"] = executed;
        response = QJsonDocument(j).toJson();
        docret = QJsonDocument::fromJson(response);
        return docret;
        break;

    case Command::LoadTool:  
        tool = rp.loadTool(doc);
        // Response Body    
        j["Request"] = "Load Tool";
        j["Response"] = tool;
        response = QJsonDocument(j).toJson();
        docret = QJsonDocument::fromJson(response);
        return docret;
        break;
    case Command::CloseAll:
        rp.closeAll();
        rn->exit();
        // Response Body
        j["Request"] = "Close All";
        j["Response"] = true;
        response = QJsonDocument(j).toJson();
        docret = QJsonDocument::fromJson(response);
        return docret;
        break;

    default:
        return QJsonDocument::fromJson(responseByte);
    }
}

QMap<QString, int> RosAction::getOutputStruct()
{
    QMap<QString, int> mapOfOutput;
    mapOfOutput.insert("Request", type::String);
    mapOfOutput.insert("Response", type::Bool);

    return mapOfOutput;
}


