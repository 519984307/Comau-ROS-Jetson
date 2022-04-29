#pragma once

#include "DatabaseConnection.h"
#include "dbmanager_global.h"

//! Class for handling Database Connection
/** Database Connections requires the following parameters 
@param Server name;
@param Driver (in this case 'SQL SERVER');
@param DB user;
@param DB password;
@param Database name;
@param Trusted connection flag;

After the opening of the database, QUERY execution is possible.
*/
class DBMANAGER_DB_CONNECTION_SQLSERVER_EXPORT DatabaseConnectionSqlServer : public DatabaseConnection
{
public:
	/** Constructor: initializes DatabaseConnectionSqlServer.
	@param server: Server name;
	@param driver: Database Driver (in this case 'SQL SERVER');
	@param user: DB user;
	@param password: DB password;
	@param dbname: Database name;
	@param trusted: Trusted connection flag;
	@param connName: Connection name;
	*/
	DatabaseConnectionSqlServer(const QString &server, const QString &driver, const QString &user, const QString &password, const QString &dbname, bool trusted=true,QString connName="");

	//! Set Database Parameters
	/*
	set the database parameters
	@param server: Server name;
	@param user: DB user;
	@param password: DB password;
	@param dbname: Database name;
	@param trusted: Trusted connection flag;
	*/
	void set(QString server, QString user, QString password, QString dbname, bool trusted);

	//! open the database
	/*
	Set database parameters and open database
	@param error - Message Error text
	@return DB Opening result */
	bool openDatabase(QString *error);
};

