#pragma once

#include "DatabaseConnection.h"
#include <QFile>
#include "dbmanager_global.h"

//! Class for handling Database Connection
/** Database Connections requires the following parameters 
@param Server name (in this case the SQLite db path);

After the opening of the database, QUERY execution is possible.
*/
class DBMANAGER_DB_CONNECTION_SQLITE_EXPORT DatabaseConnectionSqlite : public DatabaseConnection
{
public:
	//! Constructor: initializes DatabaseConnectionSqlite.
	/*
	@param server: Server name;
	@param connName: Connection name;
	*/
	DatabaseConnectionSqlite(const QString &server,QString connName="");

	//! Set Database Parameters
	/*
	* set the database parameters
	@param server: server name
	@param user: user name
	@param password: user password
	@param dbname: database name
	@param trusted: database connection is trusted
	*/
	void set(QString server, QString user, QString password, QString dbname, bool trusted);

	//! Set database parameters and open database
	 /*
	@param error: Message Error text
	@return DB Opening result */
	bool openDatabase(QString *error);
};

