#include "ROSPlan.h"


bool ROSPlan::launchUploadRobotDescription(QProcess* p)
{
    nh.deleteParam("/robot_description");
    nh.deleteParam("/robot_description_semantic");
    p->start("xterm -hold -e roslaunch racer5-0-80_description racer5-0-80_upload.launch");
    int count = 0;
    while(!nh.hasParam("/robot_description") && count < maxAttempts)
    {
        ROS_INFO_STREAM("Try loading robot description...");
        ros::Duration(1).sleep();
        count++;
    }
    //p->terminate();

    if(nh.hasParam("/robot_description"))
    {
        qDebug() << "Robot description uploaded";
        return true;
    }
    else
    {
        qDebug() << "Failed loading robot description";
        return false;
    }
}

bool ROSPlan::launchMoveItFrameWork(QProcess* p)
{
    p->start("xterm -hold -e roslaunch racer5-0-80_moveit_config demo.launch load_robot_description:=false");	
	qDebug() << "MoveIt framework launched";
    return true;
}

bool ROSPlan::launchBringup(QProcess* p)
{
    p->start("xterm -hold -e roslaunch comau_bringup racer5-0-80_bringup.launch");
    qDebug() << "Connected to Robot.";
    return true;
}

bool ROSPlan::connectToRobot()
{
    bool flag1 = launchUploadRobotDescription(launch_upload_description);
    bool flag = launchBringup(launch_bringup);
    return true;
}

bool ROSPlan::launchActionHandler(QProcess *p)
{  
    qDebug() << "Action Server launched";
    p->start("xterm -hold -e roslaunch comau_action_handlers main_comau_plan_action_handlers.launch");
    return true;
}

bool ROSPlan::launchMoveitSimple(QProcess *p)
{
    p->start("xterm -hold -e roslaunch comau_moveit_interface racer5-0-80_moveit.launch");
    qDebug() << "MoveIt launched.";
    return true;
}

bool ROSPlan::launchRViz(QProcess *p)
{
    p->start("roslaunch comau_viz rviz.launch config:=moveit");    
    qDebug() << "RViz opened.";
    return true;
}

bool ROSPlan::initFramework()
{
    bool robot_description = launchUploadRobotDescription(launch_upload_description);
    bool moveit = launchMoveItFrameWork(launch_moveit_framework);
    int count = 0;
    while(!nh.hasParam("/robot_description_semantic") && count < maxAttempts)
    {
        qDebug() << "Searching for robot_description and robot_description_semantic...";
        ros::Duration(1).sleep();
    }
    if(nh.hasParam("/robot_description_semantic"))
    {
        qDebug() << "Framework initialized.";
        bool action_server = launchActionHandler(launch_action_handlers);
        return true;
    }
    else
    {
        qDebug() << "Framework initialization failed.";
        return false;
    }
}

bool ROSPlan::initRealRobot()
{   
    bool moveit = launchMoveitSimple(launch_moveit_simple);
    bool rviz = launchRViz(launch_rviz);
    int count = 0;
    while(!nh.hasParam("/robot_description_semantic") && count < maxAttempts)
    {
        qDebug() << "Searching for robot_description and robot_description_semantic...";
        ros::Duration(1).sleep();
    }
    if(nh.hasParam("/robot_description_semantic"))
    {
        qDebug() << "Framework initialized.";
        bool action_server = launchActionHandler(launch_action_handlers);
        return true;
    }
    else
    {
        qDebug() << "Framework initialization failed.";
        return false;
    }
}

bool ROSPlan::loadTool(QJsonDocument doc)
{
    QJsonObject d = doc.object();
    bool load = d.value("load").toBool();
    nh.setParam("/load_gripper",load);
    if (load)
    {
        // Parsing Json
        gripper g;
        g.x = d.value("xg").toDouble();
        g.y = d.value("yg").toDouble();
        g.z = d.value("zg").toDouble();
        g.r = d.value("rg").toDouble();
        g.p = d.value("pg").toDouble();
        g.yw = d.value("ywg").toDouble();
        g.cad_path = d.value("cad_path").toString();
        g.name = d.value("name").toString();
        g.tool_path = d.value("tool_path").toString();

        tool t;
        t.x = d.value("xt").toDouble();
        t.y = d.value("yt").toDouble();
        t.z = d.value("zt").toDouble();
        t.r = d.value("rt").toDouble();
        t.p = d.value("pt").toDouble();
        t.yw = d.value("ywt").toDouble();

        // Generate .yaml tool config file (TODO: save in DB?)
        QString file_path = g.tool_path + g.name + ".yaml";
        QString gripper_text_format = "prefix: \"\" \nstl_file_name: \"" + g.cad_path + "\" \nx_gripper: " + QString::number(g.x, 'g', 6) + "\ny_gripper: " + QString::number(g.y, 'g', 6) + "\nz_gripper: " + QString::number(g.z, 'g', 6) + "\nroll_gripper: " + QString::number(g.r, 'g', 6) + "\npitch_gripper: " + QString::number(g.p, 'g', 6) + "\nyaw_gripper: " + QString::number(g.yw) + "\nparent_gripper: \"link_6\" \nchild_gripper: \"gripper\" \nx_ee: " + QString::number(t.x, 'g', 6) + "\ny_ee: " + QString::number(t.y, 'g', 6) + "\nz_ee: " + QString::number(t.z, 'g', 6) + "\nroll_ee: " + QString::number(t.r, 'g', 6) + "\npitch_ee: " + QString::number(t.p, 'g', 6) + "\nyaw_ee: " + QString::number(t.yw, 'g', 6) + "\nparent_ee: \"gripper\" \nchild_ee: \"ee_link\"";
        QFile file(file_path);
        if (!file.open(QFile::WriteOnly | QFile::Text))
            throw "Unable to open .yaml file";
        QTextStream out(&file);
        out << gripper_text_format;
        file.flush();
        file.close();
        // Load on ROS Parameter Server
        nh.setParam("/gripper_yaml_file",file_path.toStdString());
        qDebug() << "Robot tool loaded.";
        return true;
    }
    else
    {
        qDebug() << "No tool specified. Default tool will be loaded.";
        return true;
    }
    return false;
}

void ROSPlan::closeAll()
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
    QProcess* p = new QProcess();
    p->start("pkill RosService");
    p->start("pkill rosmaster");
}
