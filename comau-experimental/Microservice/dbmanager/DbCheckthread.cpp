#include "DbCheckthread.h"
using namespace Exceptions;

DBCheckThread::DBCheckThread(QObject *parent, DatabaseConnection*dbc)
	: QThread(parent)
{
	mTimer = new QTimer(this);
	mDatabase = dbc;
	servName = dbc->getServerName();
	dbName = dbc->getDatabaseName();
	us = dbc->getUser();
	pass = dbc->getPassword();
	isTrust = dbc->isTrusted();
	connect(this, SIGNAL(start_timer()), this, SLOT(slot_timer_start()));
	//cLog =  new Logger();
	stopFlag = false;
}

DBCheckThread::~DBCheckThread()
{
	disconnect(this, SIGNAL(start_timer()), this, SLOT(slot_timer_start()));
	delete mTimer;
}

void DBCheckThread::stopCheck()
{
	stopFlag = true;
	emit stopSignal();
}
void DBCheckThread::slot_timer_start()
{
	mTimer->start(15000);
}
void DBCheckThread::queryCheck()
{

	bool dbFlag = false;
	QString err;
	try
	{
		dbFlag = mDatabase->openDatabase(&err);
	}
	catch (DbConnectionFailedException(&(error)))
	{
		//(*cLog)(Logger::Type::Error, Level1::System, Level2::Application) << ("DB Query Error " + err);
		dbFlag = false;
	}
	emit setDBFlag(dbFlag);
}
void DBCheckThread::run()
{
	QEventLoop wait;
	connect(mTimer, SIGNAL(timeout()), &wait, SLOT(quit()));
	connect(this, SIGNAL(stopSignal()), &wait, SLOT(quit()));
	stopFlag = false;
	while (stopFlag == false) {
		queryCheck();
		emit start_timer();
		wait.exec();
	}
	disconnect(mTimer, SIGNAL(timeout()), &wait, SLOT(quit()));
	disconnect(this, SIGNAL(stopSignal()), &wait, SLOT(quit()));
	emit processEnd();
}
