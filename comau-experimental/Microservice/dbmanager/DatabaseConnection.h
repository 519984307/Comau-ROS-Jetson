#pragma once

#include <QObject>
#include <QSqlDatabase>
#include <QSqlQuery>
#include <QSqlError>
#include <QDebug>
#include <QMutex>
#include "DbManagementExceptions.h"
#include "DbType.h"
#include "dbmanager_global.h"

//! Class for handling Database Connection
/*! Database Connections requires the following parameters 
@param Server name;
@param Driver;
@param DB user;
@param DB password;
@param Database name;
@param Trusted connection flag;

After the opening of the database, QUERY execution is possible.
*/
class DBMANAGER_DB_CONNECTION_EXPORT DatabaseConnection
{
public:
	//!Constructor: initializes DatabaseConnection.
	/** Constructor: initializes DatabaseConnection.
	@param server: Server name;
	@param driver: Database Driver (in this case 'SQL SERVER');
	@param user: DB user;
	@param password: DB password;
	@param dbname: Database name;
	@param trusted: Trusted connection flag;
	@param connName: Connection name;
	*/
	DatabaseConnection(const QString &server, const QString &driver, const QString &user, const QString &password, const QString &dbname, bool trusted=true,QString connName="",DbType dbType = DbType::SQL_Server);
	//! Default destructor
	~DatabaseConnection();
	//! Set Database Parameters
	/*
	@param server: Server name;
	@param user: DB user;
	@param password: DB password;
	@param dbname: Database name;
	@param trusted: Trusted connection flag;
	*/
	virtual void set(QString server, QString user, QString password, QString dbname, bool trusted) = 0;

	//!Check if DB is open
	/** 
	@return Database Opening Flag
	*/
	bool isOpen();

	//! Get Server Name
	/*
	@return server name
	 */
	QString getServerName();

	//! Get DB User
	/*
	* @return db user
	*/
	QString getUser();

	//!Get DB Password
	/*
	 @return db user password
	*/
	QString getPassword();

	//! Get Database Name
	/*
	@return database name
	*/
	QString getDatabaseName();

	//! Get Trusted Connection Flag
	/*
	@return if the db connection is trusted
	*/
	bool isTrusted();

	/** Check if some DB Parameters are empty*/
	bool componentMissing();

	/** Set database parameters and open database
	@param error - Message Error text
	@return DB Opening result */
	virtual bool openDatabase(QString *error) = 0;

	//! Close Database
	/*! Close Database conenction*/
	void closeDatabase();

	//! start Database transaction
	void startTransaction() { mDatabase.transaction(); };

	//! end Database transaction
	void endTransaction() { mDatabase.commit(); };

	//! rollback Database transaction
	void roolbackTransaction() { mDatabase.rollback(); };


	//! get the dbType
	/*! @return db type*/
	DbType getDbType() { return dbType; };

protected:
	QSqlDatabase mDatabase; /*!< It handles a connection to a database*/
	QString mServer; /*!< Server Name*/
	QString mDriver=""; /*!< The driver used to access the database connection*/
	QString mUser=""; /*!< DB %User */
	QString mPassword=""; /*!< DB Password */
	QString mDatabaseName=""; /*!< Database name */
	bool trustedConnection; /*!< Trusted connection flag*/
	QString connectionName; /*!< Name of DB Connection */
	DbType dbType;
};

