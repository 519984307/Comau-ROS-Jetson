#pragma once
#include <QDialog>
#include <QFile>
#include "DatabaseConnection.h"
#include <QMessageBox>

#include "dbmanager_global.h"
#include <QtCore/qsettings.h>

class DbDialogPrivate;

//! Dialog for Database Connection Setup
/** After inserting DB Connection parameters, the %user can check the connection*/
class DBMANAGER_DB_DBDIALOG_EXPORT DBDialog : public QDialog
{
	Q_OBJECT

public:
	/** Constructor: initializes DBDialog.
	@param parent - Object parent
	@param dbconn - DatabaseConnection handle
	*/
	DBDialog(QWidget * parent = Q_NULLPTR, DatabaseConnection *dbconn = nullptr, QString generalSettingsPath="");
	/** Default destructor
	*/
	~DBDialog();
public slots:
	/** Check the inserted parameters. A custom waitspinner dialog is shown before connection result */
	void DBConnect();
	/** Take the DB Connection result, close the wait dialog and update DB Parameters
	@param connflag - DB Connection result flag
	*/
	//void connResultShow(bool connflag);
signals:
	/** Signal to notify DB Connection result*/
	void signal_dbConnected(DatabaseConnection*,bool);
	/** Signal to notify MainWindow closing*/
	void closeApp_signal();
protected:
	/** Event to manage Dialog and MainWindow Closing using a boolean flag*/
	void closeEvent(QCloseEvent * e);
private:
	DbDialogPrivate* dbDialogPrivate; /*!< %User Interface Object */
	QFile *DBStyle = new QFile("./Resources/qss/dbparam.qss"); /*!< Dialog Stylesheet */
	DatabaseConnection *mDB; /*!< DatabaseConnection handle */

	bool closeApp = true; /*!< Flag to force MainWindow closing */
	QString err;
	bool conection_result = false;
	QSettings* settings; /*!< QSettings handle */
};
