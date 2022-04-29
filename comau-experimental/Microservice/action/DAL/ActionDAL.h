#pragma once

#include <DatabaseConnection.h>
#include "../Action.h"
#include "DbManagementExceptions.h"
namespace DAL
{
	class ActionDAL
	{

	private:
		DatabaseConnection* dbConnection = nullptr;

	public:
		ActionDAL() {};
		/** Constructor
		 @param dbConnection the database connection
		*/
		ActionDAL(DatabaseConnection* dbConnection):dbConnection(dbConnection) {};
		/** delete all rows into Action table
		@param id: row of the  action */

		virtual bool delete_(QString name) = 0;
		/** get  action from  Action table
		@param name: db name of the selected  action*/

		virtual Action* get(QString name) = 0;
		/** add action into Action table
		@param Action-  action struct*/
		virtual bool add(Action* action) = 0;
		/** get all action from AcquisitionAction table*/
		virtual QList<Action*> getAll() = 0;

		/* get the names of all the rows*/
		virtual QList<QString> getAllNames() = 0;

		/**
		update the row associated to the name with the action params
		@params name: name of the action to update
		@params action action to update
		*/
		virtual bool update(QString name, Action* action) = 0;

	};
}
