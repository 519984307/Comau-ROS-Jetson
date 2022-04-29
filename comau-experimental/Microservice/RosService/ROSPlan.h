#pragma once
#include <QtCore/qstring.h>
#include <QtCore/qlist.h>
#include <QtCore/qdebug.h>
#include <QtCore/qprocess.h>
#include <QtCore/qjsondocument.h>
#include <QtCore/qjsonobject.h>
#include "utils.h"
#include "qfile.h"
#include "ros/ros.h"
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <QThread>
#include "QDebug"
class ROSPlan
{

private:
    // Common
	QProcess* launch_upload_description = new QProcess;
	QProcess* launch_load_tool = new QProcess;
    QProcess* launch_load_scene = new QProcess;
    QProcess* launch_comau_plan = new QProcess;
    QProcess* launch_action_handlers = new QProcess;

    // Simulation
	QProcess* launch_moveit_framework = new QProcess;

    // Real robot
    QProcess* launch_bringup = new QProcess;
    QProcess* launch_comau_action_handlers = new QProcess;
    QProcess* launch_moveit_simple = new QProcess;
    QProcess* launch_execute_trajectory = new QProcess;
    QProcess* launch_rviz = new QProcess;

    ros::NodeHandle nh;

public:
    ROSPlan(){};

    ~ROSPlan()
    {
        // Common
        launch_upload_description->terminate();
        launch_load_tool->terminate();
        launch_load_scene->terminate();
        launch_comau_plan->terminate();
        launch_action_handlers->terminate();

        // Simulation
        launch_moveit_framework->terminate();

        // Real robot
        launch_bringup->terminate();
        launch_comau_action_handlers->terminate();
        launch_moveit_simple->terminate();
        launch_execute_trajectory->terminate();
        launch_rviz->terminate();
    }
    int maxAttempts = 10;

    // Common
	bool launchUploadRobotDescription(QProcess* p);
	bool loadTool(QJsonDocument doc);
    bool launchActionHandler(QProcess* p);

    // Simulation
	bool launchMoveItFrameWork(QProcess* p);
    bool initFramework();

    // Real robot
    bool launchBringup(QProcess* p);
    bool connectToRobot();
    bool launchComauActionHandlers(QProcess* p);
    bool launchMoveitSimple(QProcess* p);
    bool launchRViz(QProcess* p);
    bool initRealRobot();

    void closeAll();

};

