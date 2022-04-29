
#ifndef EXCEPTIONS_DBMANAGEMENTEXCEPTIONS_H
#define EXCEPTIONS_DBMANAGEMENTEXCEPTIONS_H
#pragma once

#include <exception>
#include <string>
#include "dbmanager_global.h"

using namespace std;

namespace Exceptions
{
	//! Class for the management of database connection exception
	class DBMANAGER_DB_EXCEPTIONS_EXPORT DbConnectionFailedException : public exception
	{
	private:
		string errorMsg; /*!< error message */

	public:
		//! Constructor for the management of database connection exception
		/*! @param errorMsg: message error*/
		DbConnectionFailedException(char* errorMsg) :
			errorMsg(errorMsg)
		{}
		//! Destructor for the management of database connection exception
		DbConnectionFailedException()
		{}

		//! return the error message
		/*! @return error message*/
		virtual const char* what() const throw()
		{
			string*  message = new string("Couldn't connect to Precog database: ");
			return message->c_str();
		}
	};

	class DbQueryNotExecutedException : public exception
	{
	private:
		string failedQuery; /*!< query failed */

	public:
	//! Constructor for the management of the query exception
	/*! @param -failedQuery: query failed*/
		DbQueryNotExecutedException(char* failedQuery) :
			failedQuery(failedQuery)
		{}
		DbQueryNotExecutedException(string failedQuery) :
			failedQuery(failedQuery)
		{}
		  //! Default Constructor
		DbQueryNotExecutedException()
		{}
		/*! return the failed query
		@return failed query*/
		virtual const char* what() const throw()
		{
			string*  message = new string("Query failed");
			*message = message->append(failedQuery);
			return message->c_str();
		}
	};

	class DbRecordNotFoundException : public exception
	{
	private:
		string tableName; /*!< database table name  */
		string searchParameterName; /*!<search parameter name */
		string searchParameterValue; /*!< search parameter value */

	public:
		//! Constructor for the management of the failed record request
		//! @param -tableName: database table name
		//! @param -searchParameterName: search parameter name 
		//! @param -searchParameterValue: search parameter value
		DbRecordNotFoundException(char* tableName, char* searchParameterName, char* searchParameterValue) :
			tableName(tableName),
			searchParameterName(searchParameterName),
			searchParameterValue(searchParameterValue)
		{}
		//! Default Constructor
		DbRecordNotFoundException()
		{}
		DbRecordNotFoundException(QString tableName, QString searchParameterName, QString searchParameterValue):
		
			tableName(tableName.toStdString()),
				searchParameterName(searchParameterName.toStdString()),
				searchParameterValue(searchParameterValue.toStdString())
		{}
		//! return the parameter not found in the table
		/*! @return parameter not found in the table*/
		virtual const char* what() const throw()
		{
			string*  message = new string("Record with parameter not found in the table ");
			*message = message->append(searchParameterName);
			*message = message->append(" = ");
			*message = message->append(searchParameterValue);
			*message = message->append(" not found in table ");
			*message = message->append(tableName);
			return message->c_str();
		}
	};

	class DbRecordNotFoundExceptionNoParam : public exception
	{
	private:
		string tableName; /*!< database table name  */

	public:
		//! Constructor for the management of the not found parameters exception
	//! @param -tableName: database table name

		DbRecordNotFoundExceptionNoParam(char* tableName) :
			tableName(tableName)
		{}
		//! Default constructor
		DbRecordNotFoundExceptionNoParam()
		{}
		//! return the message No records found in table + tablename
		virtual const char* what() const throw()
		{
			string*  message = new string("Record with ");
			*message = message->append("No records found in table ");
			*message = message->append(tableName);
			return message->c_str();
		}
	};

	class DbQuerySelectMoreThanOneRecordException : public exception
	{
	private:
		string query; /*!< query failed  */

	public:
		//! Constructor called when more than one record are found
//! @param -query: query not executed
		DbQuerySelectMoreThanOneRecordException(char* query) :
			query(query)
		{}
		//! Default constructor
		DbQuerySelectMoreThanOneRecordException()
		{}
		//! return the message Query select more than one record.
		virtual const char* what() const throw()
		{
			string*  message = new string("Query ");
			*message = message->append(query);
			*message = message->append(" select more than one record.");
			return message->c_str();
		}
	};
	class ForeignKeyViolation : public exception
	{
	private:
		string query; /*!< query failed  */

	public:
		//! Constructor called when the foreign Key is violated
//! @param -query: query not executed
		ForeignKeyViolation(char* query) :
			query(query)
		{}
		//! Default constructor
		ForeignKeyViolation()
		{}
		//! return the message Query select more than one record.
		virtual const char* what() const throw()
		{
			string* message = new string("FOREIGN KEY constraint failed");
			
			return message->c_str();
		}
	};
	class PrimaryKeyViolation : public exception
	{
	private:
		string query; /*!< query failed  */

	public:
		//! Constructor called when the foreign Key is violated
//! @param -query: query not executed
		PrimaryKeyViolation(char* query) :
			query(query)
		{}
		//! Default constructor
		PrimaryKeyViolation()
		{}
		//! return the message Query select more than one record.
		virtual const char* what() const throw()
		{
			string* message = new string("Query ");
			*message = message->append(query);
			*message = message->append("  violates the foreign key.");
			return message->c_str();
		}
	};
}
#endif // !EXCEPTIONS_DBMANAGEMENTEXCEPTIONS_H
