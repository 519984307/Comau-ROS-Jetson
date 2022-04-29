#pragma once
/* credit: Ileana Romata*/

#include "action_global.h"
#include "qexception.h"
#include <QtCore/qjsondocument.h>
#include "httpClient.h"
#include "qimage.h"

#include <QtCore/qsavefile.h>
#include "qbuffer.h"

using namespace qtWebApp;
/* Exception of the action. Use it to throw the exception*/

class ACTION_EXPORT ActionException : public QException
{
public:
	/*Constructor 
	@param errorMsg= error message that you want to throw
	@param statusCode HttpResponseStatusCode to send*/
	ActionException(QString errorMsg, int statusCode) :errorMsg(errorMsg), statusCode(statusCode) {};
	ActionException(QString errorMsg, HttpResponseStatusCode statusCode) :errorMsg(errorMsg), statusCode(static_cast<int>(statusCode)) {};
	ActionException(int statusCode,QString errorMsg) :errorMsg(errorMsg), statusCode(statusCode) {};
	ActionException(HttpResponseStatusCode statusCode,QString errorMsg) :errorMsg(errorMsg), statusCode(static_cast<int>(statusCode)) {};
	void raise() const { throw* this; }
	ActionException* clone() const { return new ActionException(*this); }
	int getStatusCode() { return statusCode; }
	QString getErrorMsg() { return errorMsg; }
private:
	int statusCode;
	QString errorMsg;
};
/*
class for the type definition.
It is a derived of QVariant so you have 2 enum: one is QVariant::Type and the other id CustomType
*/
class ACTION_EXPORT type :public QVariant
{
public:

	enum CustomType
	{
		baseType,
		DysparityMap=100,
		Image2D=102,
		Image3D=103,

	};
	/*
	* transform the type to name
	*/
	QString getTypeToName(int type)
	{
		if (type < 100)
			QVariant::typeToName(type);
		else
		{
			
			switch (type)
			{
			case CustomType::DysparityMap:
				return "DysparityMap";
				break;
			case CustomType::Image2D:
				return "Image2D";
				break;
			case CustomType::Image3D:
				return "Image3D";
				default:
					break;
			
			}
		}
	}
	/*
* transform the name to type
*/
	int getNameToType(const char* type)
	{
		if (QVariant::nameToType(type) == Invalid)
		{

			if (type == "DysparityMap")
				return CustomType::DysparityMap;
			if (type == "Image2D")
				return CustomType::Image2D;
			if (type == "Image3D")
				return CustomType::Image3D;			
		}
		else
			return QVariant::nameToType(type);
	}

};

 class CommandRequest
{
public:
	enum commandRequest
{
	Get,
	Post,
	Head,
	Put,
	Delete,
	Custom,
	Unknown
};
	static QNetworkAccessManager::Operation fromCustomCommandToQtCommand(commandRequest request)
	{
		switch (request)
		{
		case Get:
			return QNetworkAccessManager::Operation::GetOperation;
		case Post:
			return QNetworkAccessManager::Operation::PostOperation;
		case Head:
			return QNetworkAccessManager::Operation::HeadOperation;
		case Put:
			return QNetworkAccessManager::Operation::PutOperation;
		case Delete:
			return QNetworkAccessManager::Operation::DeleteOperation;
		case Custom:
			return QNetworkAccessManager::Operation::CustomOperation;
		case Unknown:
			return QNetworkAccessManager::Operation::UnknownOperation;
		default:
			return QNetworkAccessManager::Operation::UnknownOperation;
		}
	}
};
class ACTION_EXPORT Action
{
public:

	
	 Action();
	 Action(const Action& C) { *this = C; };
	 ~Action() {};

	/*! run the action
	 @param resultOfPreviousAction: json with the input 
	 {
	 nameOfInput:Value
	 }
	 @return json with the result 
	 {
	 nameOfOutput: value
	 }
	*/
	virtual QJsonDocument run(QJsonDocument resultOfPreviousAction)=0;

	/*! pqarse the input of the action
 @param resultOfPreviousAction: json with the input given by the action executed byfore
 {
 nameOfInput:Value
 }
set your input parameter 
*/
	virtual void parseInput(QJsonDocument* resultOfPreviousAction)=0;

	/*! get the Input struct for the central db 
	 @return map with the the pair value InputName-Type of data
	*/
	virtual QMap<QString, int> getInputStruct()=0;

	/*! get the Output struct for the central db
 @return map with the the pair value OutputName-Type of data
 Attention see the Type in the action
*/
	virtual QMap<QString, int> getOutputStruct()=0;

	/*!
	* 
	*/
	QByteArray sendHttpRequest(HttpClient* httpClient,CommandRequest::commandRequest command, QMap<QString, QString> paramsMap,QString path="", QByteArray body = NULL);

	
	virtual Action* clone() const=0;
	/*return the actionName*/
	virtual QString getActionName()=0;

};
