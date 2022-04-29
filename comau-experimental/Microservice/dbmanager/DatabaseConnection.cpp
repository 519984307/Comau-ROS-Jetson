#include "DatabaseConnection.h"

using namespace Exceptions;

DatabaseConnection::DatabaseConnection(const QString &server,
	const QString &driver,
	const QString &user, 
	const QString &password,
	const QString &dbname,
	bool trusted,
	QString connName, 
	DbType dbType)
{
	connectionName = connName;
	mServer = server;
	mDriver = driver;
	mUser = user;
	mPassword = password;
	mDatabaseName = dbname;
	trustedConnection = trusted;
	this->dbType = dbType;
}

DatabaseConnection::~DatabaseConnection()
{
	closeDatabase();
	if (!connectionName.isEmpty())
		mDatabase.removeDatabase(connectionName);
	else
		QSqlDatabase::removeDatabase(mDatabase.connectionName());
}

bool DatabaseConnection::isOpen()
{
	return mDatabase.isOpen();
}

QString DatabaseConnection::getServerName()
{
	return mServer;
}

QString DatabaseConnection::getUser()
{
	return mUser;
}

QString DatabaseConnection::getPassword()
{
	return mPassword;
}

QString DatabaseConnection::getDatabaseName()
{
	return mDatabaseName;
}

bool DatabaseConnection::isTrusted()
{
	return trustedConnection;
}

bool DatabaseConnection::componentMissing()
{
	if (mServer.isEmpty() || mUser.isEmpty() || mDatabaseName.isEmpty())
		return true;
	else
		return false;
}

bool DatabaseConnection::openDatabase(QString *error)
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


void DatabaseConnection::closeDatabase()
{
	mDatabase.close();
}


