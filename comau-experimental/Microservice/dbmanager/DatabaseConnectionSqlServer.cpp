#include "DatabaseConnectionSqlServer.h"

using namespace Exceptions;

DatabaseConnectionSqlServer::DatabaseConnectionSqlServer(const QString &server,
	const QString &driver,
	const QString &user, 
	const QString &password,
	const QString &dbname,
	bool trusted,
	QString connName) : DatabaseConnection(server, driver, user, password, dbname, trusted, connName, DbType::SQL_Server)
{
	if (server.isEmpty() || driver.isEmpty() || user.isEmpty() || dbname.isEmpty())
	{
		qDebug() << "Cannot create DB Connection. Check connection parameters!";
		return;
	}
	if (connectionName.isEmpty())
		mDatabase = QSqlDatabase::addDatabase("QODBC");
	else
		mDatabase = QSqlDatabase::addDatabase("QODBC", connectionName);
}

void DatabaseConnectionSqlServer::set(QString server,QString user,QString password, QString dbname, bool trusted){
	if (server.isEmpty() || user.isEmpty() || dbname.isEmpty())
	{
		qDebug() << "Cannot create DB Connection. Check connection parameters!";
		return;
	}
	
	if (connectionName.isEmpty())
		mDatabase = QSqlDatabase::addDatabase("QODBC");
	else
		mDatabase = QSqlDatabase::addDatabase("QODBC", connectionName);
	mDriver = "SQL Server";
	mServer = server;
	mUser = user;
	mPassword = password;
	mDatabaseName = dbname;
	trustedConnection = trusted;
}

bool DatabaseConnectionSqlServer::openDatabase(QString *error)
{
	mDatabase.setDatabaseName(QString("DRIVER={%1};SERVER=%2;DATABASE=%3;UID=%4;PWD=%5;Trusted_Connection=%6;")
		.arg(mDriver)
		.arg(mServer)
		.arg(mDatabaseName)
		.arg(mUser)
		.arg(mPassword)
		.arg(trustedConnection ? "Yes" : "No"));

	if (!mDatabase.open()) {
		if (error != nullptr) {
			*error = mDatabase.lastError().text();
		}
		throw DbConnectionFailedException(&(error->toStdString())[0]);
		return false;
	}
	return true;
}