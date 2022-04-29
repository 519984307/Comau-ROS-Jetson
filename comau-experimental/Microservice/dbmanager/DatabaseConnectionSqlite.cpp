#include "DatabaseConnectionSqlite.h"

using namespace Exceptions;

DatabaseConnectionSqlite::DatabaseConnectionSqlite(const QString &server,
	QString connName) : DatabaseConnection(server, "", "", "", "", "", connName, DbType::Sqlite)
{
	if (server.isEmpty())
	{
		qDebug() << "Cannot create DB Connection. Check connection parameters!";
		return;
	}
	if (connectionName.isEmpty())
		mDatabase = QSqlDatabase::addDatabase("QSQLITE");
	else
		mDatabase = QSqlDatabase::addDatabase("QSQLITE", connectionName);
}

void DatabaseConnectionSqlite::set(QString server, QString user, QString password, QString dbname, bool trusted) {
	if (server.isEmpty())
	{
		qDebug() << "Cannot create DB Connection. Check connection parameters!";
		return;
	}

	if (connectionName.isEmpty())
		mDatabase = QSqlDatabase::addDatabase("QSQLITE");
	else
		mDatabase = QSqlDatabase::addDatabase("QSQLITE", connectionName);
	mDriver = "";
	mServer = server;
	mUser = "";
	mPassword = "";
	mDatabaseName = "";
	trustedConnection = "";
}

bool DatabaseConnectionSqlite::openDatabase(QString *error)
{
	mDatabase.setDatabaseName(mServer);
	if (QFile::exists(mServer))
	{
		try
		{
			if (!mDatabase.open())
			{
				if (error != nullptr)
				{
					*error = mDatabase.lastError().text();
				}
				throw DbConnectionFailedException(&(error->toStdString())[0]);
				return false;
			}
			QSqlQuery query;
			bool exec = query.exec("PRAGMA foreign_keys = ON;");
			return true;
		}
		catch (QSqlError* error)
		{
			return false;
		}
		catch (exception_ptr)
		{
			return false;
		}
		catch(...)
		{
			return false;
		}

	}
	else
	{
		// Db file Not found
		return false;
	}
}

