#include "RosActionDAL.h"
using namespace Exceptions;

bool RosActionDAL::delete_(QString name)
{

	bool isOpen = dbConnection->isOpen();
	if (!isOpen)
	{
		QString error;
		isOpen = dbConnection->openDatabase(&error);
		if (!isOpen)
			throw DbConnectionFailedException(&(error.toStdString())[0]);
	}
	QSqlQuery query;
	query.prepare("DELETE FROM [RosAction] WHERE Name=:name");
	query.bindValue(":name", name);
	bool executed = query.exec();
	if (!executed)
		throw DbQueryNotExecutedException(&(query.lastQuery().toStdString())[0]);

	return executed;
}

Action* RosActionDAL::get(QString name)
{

	bool isOpen = dbConnection->isOpen();
	if (!isOpen)
	{
		QString error;
		isOpen = dbConnection->openDatabase(&error);
		if (!isOpen)
			throw DbConnectionFailedException(&(error.toStdString())[0]);
	}
	QSqlQuery query;
	query.prepare("SELECT * FROM [RosAction] WHERE Name=:name");
	query.bindValue(":name", name);
	bool executed = query.exec();
	if (executed)
	{
		if (query.next())
		{

			QString command = query.value(1).toString();
			return  new RosAction(name, command);
		}
		else
			throw DbRecordNotFoundException("[RosAction]", "Name", &(name.toStdString())[0]);
	}
	else
	{
		throw DbQueryNotExecutedException(&(query.lastQuery().toStdString())[0]);
	}

}

bool RosActionDAL::add(Action* rosAction)
{

	RosAction* tmpAction = (RosAction*)rosAction;
	Command commandManager;
	QString command = commandManager.fromCommandToString(tmpAction->getCommand());
	QString name = tmpAction->getActionName();
	bool isOpen = dbConnection->isOpen();
	
	if (!isOpen)
	{
		QString error;
		isOpen = dbConnection->openDatabase(&error);
		
		if (!isOpen)
			qDebug() << "DbConnectionFailedException";
			throw DbConnectionFailedException(&(error.toStdString())[0]);
	}
	try
	{

		if (this->getAllNames().contains(rosAction->getActionName()))
			qDebug() << "Exept: PrimaryKeyViolation";
			throw PrimaryKeyViolation();
	}
	catch (DbQueryNotExecutedException) {}
	


	QSqlQuery query;
	query.prepare("INSERT INTO [RosAction] ([Name],[Command]) VALUES (:name,:command)");
	//qDebug() << "preparing query...";
	query.bindValue(":command", command);
	query.bindValue(":name", name);
	bool executed = query.exec();
	qDebug() << query.lastError();
	qDebug() << query.lastQuery();
	//QSqlError 
	if (!executed)
	{
		if (query.lastError().text().contains("FOREIGN KEY constraint failed"))
		{
			qDebug() << "ForeignKeyViolation";
			throw(ForeignKeyViolation());
		}
			
		else
			qDebug() << "Except:DbQueryNotExecutedException";
			throw DbQueryNotExecutedException(&(query.lastQuery().toStdString())[0]);
	}

	return executed;
}

QList<Action*> RosActionDAL::getAll()
{
	//qDebug() << "Sono in getAll";

	bool isOpen = dbConnection->isOpen();
	if (!isOpen)
	{
		QString error;
		isOpen = dbConnection->openDatabase(&error);
		if (!isOpen)
			throw DbConnectionFailedException(&(error.toStdString())[0]);
	}
	QSqlQuery query;
	query.prepare("SELECT * FROM [RosAction]");
	bool executed = query.exec();

	if (executed)
	{
		//qDebug() << "Query getAll executed";
		QList <Action*> rosActionList;
		while (query.next())
		{
			QString name = query.value(0).toString();
			QString command = query.value(1).toString();
			rosActionList.append(new RosAction(name, command));
		}
		//qDebug() << "ActionList:" << rosActionList;
		return rosActionList;
	}
	else
	{
		throw DbQueryNotExecutedException(&(query.lastQuery().toStdString())[0]);
	}

}

QList<QString> RosActionDAL::getAllNames()
{

	bool isOpen = dbConnection->isOpen();
	if (!isOpen)
	{
		QString error;
		isOpen = dbConnection->openDatabase(&error);
		if (!isOpen)
			throw DbConnectionFailedException(&(error.toStdString())[0]);
	}
	QSqlQuery query;
	query.prepare("SELECT Name FROM [RosAction]");
	bool executed = query.exec();

	if (executed)
	{
		QList <QString> actionList;
		while (query.next())
		{
			QString name = query.value(0).toString();

			actionList.append(name);
		}
		return actionList;
	}
	else
	{
		throw DbQueryNotExecutedException(&(query.lastQuery().toStdString())[0]);
	}

}

bool RosActionDAL::update(QString nameToUpdate, Action* rosAction)
{

	RosAction* tmpAc = (RosAction*)rosAction;
	bool isOpen = dbConnection->isOpen();
	if (!isOpen)
	{
		QString error;
		isOpen = dbConnection->openDatabase(&error);
		if (!isOpen)
			throw DbConnectionFailedException(&(error.toStdString())[0]);
	}
	if (!(this->getAllNames().contains(nameToUpdate)))
		throw DbRecordNotFoundException();
	QSqlQuery query;
	query.prepare("UPDATE [rosAction] SET [Name]= :name, [Command]= :command WHERE [Name]=:nameToUpdate");

	query.bindValue(":command", tmpAc->getCommand());
	query.bindValue(":name", tmpAc->getActionName());
	bool executed = query.exec();

	if (!executed)
	{
		qDebug() << (query.lastError().text());
		if (query.lastError().text().contains("FOREIGN KEY constraint failed"))
			throw(ForeignKeyViolation());
		if (query.lastError().text().contains("UNIQUE constraint failed"))
			throw(PrimaryKeyViolation());
		else
			throw DbQueryNotExecutedException(&(query.lastQuery().toStdString())[0]);
	}

	return executed;
}
