#pragma once
#include <QObject>
#include "httpClient.h"
#include "qjsondocument.h"
#include "qjsonobject.h"
#include "qexception.h"
#include "Action.h"
#include "ROSPlan.h"
#include "rosnode.h"
using namespace qtWebApp;

class Command
{
public:
	enum command
	{
		MoveP2P,
        Test,
        Execute,
		InitFramework,
        ConnectToRobot,
        InitRealRobot,
		LoadScene,
		LoadTool,
        CloseAll,
		Default
	};
	QString fromCommandToString(command command)
	{
		switch (command)
		{
		case MoveP2P:
			return "MoveP2P";
        case Test:
            return "Test";
        case Execute:
            return "Execute";
		case InitFramework:
			return "InitFramework";
        case ConnectToRobot:
            return "ConnectToRobot";
        case InitRealRobot:
            return "InitRealRobot";
		case LoadScene:
			return "LoadScene";
		case LoadTool:
			return "LoadTool";
        case CloseAll:
            return "CloseAll";
		default:
			return "";
		}
	};
	command fromStringToCommand(QString commandString)
	{
		if (commandString.contains("MoveP2P", Qt::CaseInsensitive))
			return MoveP2P;
        if (commandString.contains("Test", Qt::CaseInsensitive))
            return Test;
        if (commandString.contains("Execute", Qt::CaseInsensitive))
            return Execute;
        if (commandString.contains("InitFramework", Qt::CaseInsensitive))
			return InitFramework;
        if (commandString.contains("ConnectToRobot", Qt::CaseInsensitive))
            return ConnectToRobot;
        if (commandString.contains("InitRealRobot", Qt::CaseInsensitive))
            return InitRealRobot;
		if (commandString.contains("LoadScene", Qt::CaseInsensitive))
			return LoadScene;
		if (commandString.contains("LoadTool", Qt::CaseInsensitive))
			return LoadTool;
        if (commandString.contains("CloseAll", Qt::CaseInsensitive))
            return CloseAll;
		return Default;
	};

};

class RosAction : public Action
{

private:
	QString name = "";
	Command::command command = Command::Default;
	Command commandManager;
    ROSPlan rp;
    ROSNode* rn;

public:
	RosAction(QString name, QString command) :name(name)
	{
		this->command = commandManager.fromStringToCommand(command);
	};

	~RosAction();
	RosAction() 
	{
        qDebug() << "Default constructor";
		name = "";
		command = Command::Default;
	};
    RosAction(ROSNode* rn) : rn(rn)
    {
        qDebug() << "Default constructor";
        name = "";
        command = Command::Default;

    };

    RosAction(const RosAction& C) { *this = C; }


	/*! run the action
@param resultOfPreviousAction: json with the input
{
nameOfInput:Value
}
@return json with the result
{
nameOfOutput: value
}
*/
    QJsonDocument run(QJsonDocument resultOfPreviousAction) override;

	void parseInput(QJsonDocument* resultOfPreviousAction) {};

	bool checkInput();

	/*! get the Input struct for the central db
	@return map with the the pair value InputName-Type of data
	*/
	QMap<QString, int> getInputStruct() override { return  QMap<QString, int>(); };

	/*! get the Output struct for the central db
	@return map with the the pair value OutputName-Type of data
	*/
	QMap<QString, int> getOutputStruct() override;


	bool create(QJsonDocument inputParameter);
	bool update(QJsonDocument inputParameter) { return true; };
	QString getActionName() { return name; };
	Command::command getCommand() { return command; };
	QString getActionType() { return ""; };
	Action* clone() const { return new RosAction(*this); }


};

