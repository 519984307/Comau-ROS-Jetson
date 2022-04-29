#include "rosnode.h"
using namespace Exceptions;

ROSNode::ROSNode(int argc, char **argv, QSettings* sett) : argc(argc), argv(argv), sett(sett)
{
    init();
    conn = getDBConnection();
}

ROSNode::~ROSNode()
{
    std::cout << "RosNOde destructor" << std::endl;
    if(ros::isStarted()){
        qDebug("ROS Node is shutting down...");
        ros::shutdown(); //
    }
    ros::shutdown();
    //wait();
    QProcess* p = new QProcess;
    p->start("pkill RosService");
    quit();
    exit();
    terminate();
}

bool ROSNode::init()
{
    ros::init(argc, argv, "RosNode");
    if ( !ros::master::check()) {
        return false;
    }
    ros::start();
    qDebug("ROS Node is running...");
    return true;
}

void ROSNode::run()
{
    // QThread function
    ros::Rate loop_rate(10); // too fast loop rate crashed the GUI
    std::cout << "Node is running." << std::endl;
    while (ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
        std::cout << "Spinno" << std::endl;
    }

}

bool ROSNode::sendTargetGoal(QJsonDocument doc)
{
    comau_msgs::MoveToPoseMoveItActionGoal msg;
    msg.goal.constraint_mode = 0;
    msg.goal.target_pose = getTarget(doc);
    msg.goal.execute = doc.object().value("execute").toBool();
    msg.goal.send_trajectory = doc.object().value("send_trajectory").toBool();
    msg.goal.timeout = doc.object().value("timeout").toDouble();
    msg.goal.planning_pipeline = doc.object().value("planning_pipeline").toString().toStdString();
    msg.goal.geometric_planner = doc.object().value("geometric_planner").toString().toStdString();
    msg.goal.velocity = 1.0;
    msg.goal.acceleration = 1.0;
    client->sendGoal(msg.goal);
    //bool finished_before_timeout = client->waitForResult(ros::Duration(msg.goal.timeout*1.5));
    bool finished_before_timeout = client->waitForResult(ros::Duration(30));
    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = client->getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
        if(state.toString() == "SUCCEEDED")
        {
            return true;
        }

    }
    else
    {
        ROS_INFO("Action did not finish before the time out.");
        return false;
    }
    return false;
}

bool ROSNode::sendTargetGoalOpt(QJsonDocument doc)
{
    // Construct msg
    comau_msgs::MoveToPoseMoveItActionGoal msg;
    msg.goal.constraint_mode = 0;
    msg.goal.target_pose = getTarget(doc);
    msg.goal.execute = doc.object().value("execute").toBool();
    msg.goal.send_trajectory = doc.object().value("send_trajectory").toBool();
    msg.goal.timeout = doc.object().value("timeout").toDouble();
    msg.goal.id = doc.object().value("id").toString().toStdString();
    msg.goal.planning_pipeline = doc.object().value("planning_pipeline").toString().toStdString();
    msg.goal.geometric_planner = doc.object().value("geometric_planner").toString().toStdString();
    msg.goal.velocity = 1.0;
    msg.goal.acceleration = 1.0;
    QString id = QString::fromStdString(msg.goal.id);
    bool is_id = getIdFromDB(id);
    // Check if <id> is in the database and get plan
    if(is_id)
    {
        msg.goal.hasPlan = true;
        msg.goal.plan = deserialize(getPlanFromDB(id)); //moveit_msgs::MotionPlanResponse
        qDebug() << "Path has been already planned and loaded from database";
    }
    else
    {
        msg.goal.hasPlan = false;
        qDebug() << "Path was not found in the database.";
    }

    // Send msg
    client->sendGoal(msg.goal);
    // Wait for result
    bool finished_before_timeout = client->waitForResult(ros::Duration(30));
    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = client->getState();
        comau_msgs::MoveToPoseMoveItResultConstPtr result;

        ROS_INFO("Action finished: %s",state.toString().c_str());
        if(state.toString() == "SUCCEEDED")
        {
            // If a plan was not previously saved in db, save it
            if(!is_id && id !="")
            {
                qDebug() << "Saving path to database with id : " << id;
                result = client->getResult();
                addPlanToDB(id,result->action_result.plan);
                qDebug() << "Path saved!";
            }
            return true;
        }
    }
    else
    {
        ROS_INFO("Action did not finish before the time out.");
        return false;
    }
    return false;
}

bool ROSNode::execute(QJsonDocument doc)
{
    comau_msgs::MoveToPoseMoveItActionGoal msg;
    msg.goal.send_trajectory = doc.object().value("send_trajectory").toBool();
    msg.goal.execute = doc.object().value("execute").toBool();
    msg.goal.hasPlan = false;
    client->sendGoal(msg.goal);
    bool finished_before_timeout = client->waitForResult(ros::Duration(5));
    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = client->getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
        if(state.toString() == "SUCCEEDED")
        {
            qDebug() << "Path executed";
            return true;
        }

    }
    else
    {
        ROS_INFO("Action did not finish before the time out.");
        return false;
    }
    return false;

}

geometry_msgs::PoseStamped ROSNode::getTarget(QJsonDocument doc)
{
    target.header.frame_id = "base_link";
    target.pose.position.x = doc.object().value("x").toDouble();
    target.pose.position.y = doc.object().value("y").toDouble();
    target.pose.position.z = doc.object().value("z").toDouble();
    target.pose.orientation.w = doc.object().value("w").toDouble();
    target.pose.orientation.x = doc.object().value("qx").toDouble();
    target.pose.orientation.y = doc.object().value("qy").toDouble();
    target.pose.orientation.z = doc.object().value("qz").toDouble();
    return target;
}

void ROSNode::initClient()
{
    // Plan & Execute
    client.reset(new Client("move_topose_handler_server/action", true));
    ROS_INFO("Waiting for Comau Plan Action Server to start.");
    client->waitForServer(); //wait for client to connect to server
    ROS_INFO("Action server started, sending goal.");

    // Publish PCL
    client_p.reset(new ClientPerception("perception_handler_server/action", true));
    ROS_INFO("Waiting for Comau Perception Server to start.");
    client_p->waitForServer(); //wait for client to connect to server
    ROS_INFO("Action server started, sending goal.");

}

bool ROSNode::publishPCL(QJsonDocument doc)
{
    QProcess* p = new QProcess;
    ROS_INFO("Clearing Octomap first");
    p->start("rosservice call /clear_octomap");
    p->waitForFinished(10);
    ROS_INFO("Octomap cleared. Reset RViz.");
    msg_p.goal.path = doc.object().value("path").toString().toStdString();
    msg_p.goal.x = doc.object().value("x").toDouble();
    msg_p.goal.y = doc.object().value("y").toDouble();
    msg_p.goal.z = doc.object().value("z").toDouble();
    msg_p.goal.r = doc.object().value("r").toDouble();
    msg_p.goal.p = doc.object().value("p").toDouble();
    msg_p.goal.yw = doc.object().value("yw").toDouble();
    msg_p.goal.res = doc.object().value("res").toDouble();
    client_p->sendGoal(msg_p.goal);

    bool finished_before_timeout = client_p->waitForResult(ros::Duration(10.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = client_p->getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
        return true;
    }
    else
        ROS_INFO("Action did not finish before the time out.");
        return false;
}

QSqlDatabase ROSNode::getDBConnection()
{
    // Starting with Qt 5.11, sharing the same connection between threads is not allowed.
    // Use a dedicated connection for each thread requiring access to the database,
    // using the thread address as connection name.

    QSqlDatabase cnx;

    QString dbName = QStringLiteral("myConnection_%1").arg(qintptr(QThread::currentThreadId()), 0, 16);
    if(QSqlDatabase::contains(dbName))
    {
        cnx = QSqlDatabase::database(dbName);
    }
    else
    {
        //QString m_dbPath = "/home/antonio/Lib/RosService/DAL/RosAction.sqlite";
        m_dbPath = sett->value("dbPath", "").toString();
        cnx = QSqlDatabase::addDatabase(QStringLiteral("QSQLITE"), dbName);
        cnx.setDatabaseName(m_dbPath);
        cnx.setConnectOptions("QSQLITE_BUSY_TIMEOUT=1000");
        if (!cnx.isValid() || !cnx.open())
        {
            qDebug() << "DB connection creation error!";
        }
        else
        {
             qDebug() << "DB connection creation succeded!";
        }
    }
    return cnx;
}

bool ROSNode::addPlanToDB(QString id, moveit_msgs::MotionPlanResponse plan)
{

    //QSqlDatabase conn = getDBConnection();
    bool isOpen = conn.open();
    // insert except handl
    //
    //

    QSqlQuery query(conn);
    query.prepare("INSERT INTO [Paths] ([id],[plan]) VALUES (:id,:plan)");
    query.bindValue(":id", id);
    QByteArray plan_ser = serialize(plan);
    query.bindValue(":plan", plan_ser);
    bool executed = query.exec();
    qDebug() << query.lastError();
    qDebug() << query.lastQuery();
    //QSqlError
    if (!executed)
    {
        if (query.lastError().text().contains("FOREIGN KEY constraint failed"))
        {
            throw(ForeignKeyViolation());
        }

        else
            throw DbQueryNotExecutedException(&(query.lastQuery().toStdString())[0]);
    }

//    moveit_msgs::MotionPlanResponse tmp_des = deserialize(getPlanFromDB(id));
//    ROS_INFO_STREAM("Deserialized plan: " << tmp_des);
}

QByteArray ROSNode::serialize(moveit_msgs::MotionPlanResponse plan)
{
    namespace ser = ros::serialization;
    uint32_t serial_size = ros::serialization::serializationLength(plan);
    boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);

    ser::OStream stream(buffer.get(), serial_size);
    ser::serialize(stream, plan);

    QByteArray dataArr;
    dataArr.resize(serial_size);
    for(unsigned int i = 0; i < serial_size; i++)
    {
        dataArr[i] = buffer[i];
    }
    return dataArr;
}

moveit_msgs::MotionPlanResponse ROSNode::deserialize(QByteArray dataArr)
{
    namespace ser = ros::serialization;

    moveit_msgs::MotionPlanResponse plan;
    uint32_t serial_size = dataArr.size();
    boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
    for(unsigned int i = 0; i < serial_size; i++)
    {
        buffer[i] = dataArr[i];
    }
    // Fill buffer with a serialized UInt32
    ser::IStream stream(buffer.get(), serial_size);
    ROS_INFO("Deserializing...");
    ros::serialization::Serializer<moveit_msgs::MotionPlanResponse>::read(stream, plan);
    ROS_INFO("Deserialized");
    return plan;
}

QByteArray ROSNode::getPlanFromDB(QString Id)
{
    //QSqlDatabase conn = getDBConnection();
    bool isOpen = conn.open();
    QSqlQuery query(conn);
    query.prepare("SELECT * FROM [Paths] WHERE id=:Id");
    query.bindValue(":Id", Id);
    bool executed = query.exec();
    if (executed)
    {
        if (query.next())
        {

            QByteArray plan = query.value(1).toByteArray();
            return  plan;
        }
        else
            throw DbRecordNotFoundException("[Paths]", "id", &(Id.toStdString())[0]);
    }
    else
    {
        throw DbQueryNotExecutedException(&(query.lastQuery().toStdString())[0]);
    }
}

bool ROSNode::getIdFromDB(QString Id)
{
    //QSqlDatabase conn = getDBConnection();
    bool isOpen = conn.open();
    QSqlQuery query(conn);
    query.prepare("SELECT * FROM [Paths] WHERE id=:Id");
    query.bindValue(":Id", Id);
    bool executed = query.exec();
    if (executed)
    {
        if (query.next())
        {
            return true;
        }
        else
        {
            return false;
        }

    }
    else
    {
        throw DbQueryNotExecutedException(&(query.lastQuery().toStdString())[0]);
    }
}
