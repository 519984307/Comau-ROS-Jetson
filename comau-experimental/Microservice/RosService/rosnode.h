#ifndef ROSNODE_H
#define ROSNODE_H

#include <ros/ros.h>
#include <ros/serialization.h>
#include <std_msgs/String.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <std_msgs/Bool.h>
#include <actionlib/client/simple_action_client.h>
#include <comau_msgs/MoveToPoseMoveItAction.h>
#include <comau_msgs/MoveToPoseMoveItActionGoal.h>
#include <comau_msgs/MoveToPoseMoveItActionResult.h>
#include <comau_msgs/MoveToPoseMoveItResult.h>
#include <comau_msgs/ActionResult.h>
#include <comau_msgs/PerceptionAction.h>
#include <comau_msgs/PerceptionActionGoal.h>
#include <geometry_msgs/PoseStamped.h>
#include <boost/shared_ptr.hpp>
#include <QtCore/qjsondocument.h>
#include <QtCore/qjsonobject.h>
#include <QtCore/qprocess.h>

#include "QSqlDatabase"
#include <DatabaseConnection.h>
#include "DbManagementExceptions.h"
#include "DAL/ActionDAL.h"
class ROSNode :public QThread
{
    Q_OBJECT
public:
    ROSNode(int argc, char** argv, QSettings* sett);
    virtual ~ROSNode();

    // Init
    bool init();
    void run();
    void initClient();

    // Point Cloud
    bool publishPCL(QJsonDocument doc);

    // Plan & Execute
    bool sendTargetGoal(QJsonDocument doc);
    bool sendTargetGoalOpt(QJsonDocument doc);
    bool execute(QJsonDocument doc);

    // Clients
    typedef actionlib::SimpleActionClient<comau_msgs::MoveToPoseMoveItAction> Client;
    typedef actionlib::SimpleActionClient<comau_msgs::PerceptionAction> ClientPerception;
    boost::shared_ptr<Client> client;
    boost::shared_ptr<ClientPerception> client_p;

private:
    int argc;
    char **argv;
    QSettings* sett;
    geometry_msgs::PoseStamped target;
    comau_msgs::PerceptionActionGoal msg_p;
    bool finished_before_timeout;
    QSqlDatabase conn;
    QString m_dbPath = "";

    geometry_msgs::PoseStamped getTarget(QJsonDocument doc);
    QSqlDatabase getDBConnection();
    bool addPlanToDB(QString id, moveit_msgs::MotionPlanResponse plan);
    QByteArray getPlanFromDB(QString Id);
    QByteArray serialize(moveit_msgs::MotionPlanResponse plan);
    moveit_msgs::MotionPlanResponse deserialize(QByteArray dataArr);
    bool getIdFromDB(QString Id);
};

#endif // ROSNODE_H
