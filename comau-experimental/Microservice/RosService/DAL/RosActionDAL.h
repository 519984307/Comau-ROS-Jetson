#pragma once
#include <DatabaseConnection.h>
#include "../RosAction.h"
#include "DbManagementExceptions.h"
#include "DAL/ActionDAL.h"

class RosActionDAL :public DAL::ActionDAL
{

private:
	DatabaseConnection* dbConnection = nullptr;

public:
	/** Constructor
	 @param dbConnection the database connection
	*/
	RosActionDAL(DatabaseConnection* dbConnection) : dbConnection(dbConnection) {};

	/** delete all rows into AcquisitionAction table
	@param id: row of the Acquisition action */

	bool delete_(QString name) override;
	/** get camera parameter from VisionSensors table
	@param name: db name of the selected acquisition action*/

	Action* get(QString name) override;
	/** add Camera into VisionSensors table
	@param AcquisitionAction- acquisition action struct*/
	bool add(Action* action);
	/** get all action from AcquisitionAction table*/
	QList<Action*> getAll() override;
	QList<QString> getAllNames() override;

	bool update(QString name, Action* object) override;

};

