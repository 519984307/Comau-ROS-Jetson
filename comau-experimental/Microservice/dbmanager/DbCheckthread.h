#ifndef DBCHECKTHREAD_H
#define DBCHECKTHREAD_H
#pragma once
#include <QThread>
#include "DatabaseConnection.h"
#include <QEventLoop>
#include <QTimer>
#include "DbManagementExceptions.h"

#include "dbmanager_global.h"

//! Thread for checking database connection 
/** This thread execute a SELECT 1 query to evaluate database connection. Query is
executed at every timeout. */
class DBMANAGER_DB_CHECKTHREAD_EXPORT DBCheckThread : public QThread
{
	Q_OBJECT

public:
	//! Constructor
	/*! initializes DBCheckThread 
	@param parent - Parent object 
	@param dbc - Database connection handle */
	DBCheckThread(QObject *parent,DatabaseConnection*dbc);
	//! Default destructor
	~DBCheckThread();
	//!stop the database connection check
	/*! Activate stop flag for thread stopping */
	void stopCheck();
	//! check if the database is conencted
	/*! execute a query and check id the database is connected  */
	void queryCheck();
	public slots:
	/** Start timer */
	void slot_timer_start();

signals:
	/** Set DB Flag
	@param flag - Database flag (0=OK,-1 NOK)*/
	void setDBFlag(bool flag);
	/** Notify thread execution end */
	void processEnd();
protected:
	/** Thread execution function */
	void run() override;
private:
	bool stopFlag; /*!< Flag for stopping thread execution*/
	QTimer* mTimer; /*!< Check timer */
	QString servName; /*!< DB Server */
	QString dbName; /*!< Database name */
	QString us; /*!< DB User */
	QString pass; /*!< DB Password */
	bool isTrust; /*!< Trusted connection flag */

	DatabaseConnection* mDatabase;
signals:
	/** Notify stop flag activation */
	void stopSignal();
	/** Activate timer starting */
	void start_timer();
};

#endif // DBCHECKTHREAD_H
