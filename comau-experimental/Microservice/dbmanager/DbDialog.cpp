#include "DbDialog.h"
#include <string.h>
#include "ui_DbDialog.h"

struct DbDialogPrivate 
{
	Ui::DbDialog ui;
};

DBDialog::DBDialog(QWidget * parent, DatabaseConnection *dbconn,QString generalSettingsPath) : QDialog(parent), dbDialogPrivate(new DbDialogPrivate)
{
	dbDialogPrivate->ui.setupUi(this);
	setWindowFlags(windowFlags() & ~Qt::WindowContextHelpButtonHint);
	mDB = dbconn;
	DBStyle->close();
	this->setWindowFlags(Qt::Window | Qt::FramelessWindowHint);
	this->setAttribute(Qt::WA_TranslucentBackground);
	
	settings = new QSettings(generalSettingsPath, QSettings::Format::IniFormat, this);
	if (mDB->getDbType() == DbType::Sqlite)
	{
		dbDialogPrivate->ui.container->setCurrentIndex(1);
		dbDialogPrivate->ui.dbPathLineEdit->setTextMargins(20, 0, 20, 0);
		dbDialogPrivate->ui.dbPathLineEdit->setText(settings->value("dbPath", "").toString());
	}
	else
	{
		dbDialogPrivate->ui.container->setCurrentIndex(0);
		dbDialogPrivate->ui.passwd_input->setEchoMode(QLineEdit::Password);
		dbDialogPrivate->ui.user_input->setTextMargins(20, 0, 20, 0);
		dbDialogPrivate->ui.passwd_input->setTextMargins(20, 0, 20, 0);
		dbDialogPrivate->ui.dbname_input->setTextMargins(20, 0, 20, 0);
		dbDialogPrivate->ui.server_input->setTextMargins(20, 0, 20, 0);
		dbDialogPrivate->ui.user_input->setText(settings->value("serverUser", "").toString());
		dbDialogPrivate->ui.passwd_input->setText("");
		dbDialogPrivate->ui.dbname_input->setText(settings->value("DBName", "").toString());
		dbDialogPrivate->ui.server_input->setText(settings->value("serverName", "").toString());
		dbDialogPrivate->ui.yesRB->setChecked(settings->value("trusted", false).toBool());
	}
	
	connect(dbDialogPrivate->ui.connect_button, SIGNAL(clicked()), this, SLOT(DBConnect()));
	connect(dbDialogPrivate->ui.connect_button_2, SIGNAL(clicked()), this, SLOT(DBConnect()));
	connect(dbDialogPrivate->ui.closeButton_2, SIGNAL(clicked()), this, SLOT(close()));
	connect(this, SIGNAL(closeApp_signal()), parentWidget(), SLOT(closeApp()));
	connect(this, SIGNAL(signal_dbConnected(DatabaseConnection*, bool)), parentWidget(), SLOT(slot_setDBConnection(DatabaseConnection*, bool)));
}

DBDialog::~DBDialog() 
{
	disconnect(dbDialogPrivate->ui.connect_button, SIGNAL(clicked()), this, SLOT(DBConnect()));
	disconnect(this, SIGNAL(closeApp_signal()), parentWidget(), SLOT(closeApp()));
	delete dbDialogPrivate;
}

// close dialog
void DBDialog::closeEvent(QCloseEvent * e)
{
		emit closeApp_signal();
}

// connet to database
void DBDialog::DBConnect() 
{
	QString serverName = dbDialogPrivate->ui.server_input->text();
	QString dbName = dbDialogPrivate->ui.dbname_input->text();
	QString dbUser = dbDialogPrivate->ui.user_input->text();
	QString dbPass = dbDialogPrivate->ui.passwd_input->text();
	bool isTrusted = dbDialogPrivate->ui.yesRB->isChecked();
	QString dbPath = dbDialogPrivate->ui.dbPathLineEdit->text();
	if(mDB->getDbType() == DbType::Sqlite)
		mDB->set(dbPath, "", "", "", false);
	else
		mDB->set(serverName, dbUser, dbPass, dbName, isTrusted);
	try
	{
		bool opened = mDB->openDatabase(&err);
		if (mDB->isOpen())
		{
			conection_result = true;
			if (mDB->getDbType() == DbType::Sqlite)
			{
				settings->setValue("dbPath", dbPath);
			}
			else
			{
				settings->setValue("serverName", serverName);
				settings->setValue("serverUser", dbUser);
				//settings->setValue("serverPassword", settMan->encrypt(dbPass));
				settings->setValue("DBName", dbName);
				settings->setValue("trusted", isTrusted);
			}
		}
		else
			conection_result = false;
	}
	catch (Exceptions::DbConnectionFailedException)
	{
		conection_result = false;
		QMessageBox::warning(this,"Error","Connection error! \n");
	}
	emit signal_dbConnected(mDB, conection_result);
}




