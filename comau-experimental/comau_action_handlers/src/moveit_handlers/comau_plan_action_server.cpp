#include "moveit_handlers/comau_plan_action_server.h"


MoveToPoseHandlerServer::MoveToPoseHandlerServer(ros::NodeHandle &nh, ros::NodeHandle &nh_local, std::string name)
    : nh_(nh), nh_local_(nh_local), action_name_(std::move(name)) 
    {
                                             
    }

MoveToPoseHandlerServer::~MoveToPoseHandlerServer(void) {}


bool MoveToPoseHandlerServer::initialize()
{
    // Get planning group from parameter server
    if (!nh_local_.getParam("move_group", PLANNING_GROUP_))
    {
        ROS_ERROR_STREAM("[ PLANNING_GROUP_ ] Required parameter " << nh_local_.resolveName("move_group") << " not given.");
        return false;
    }

    // Create move_group_ object associated to PLANNING_GROUP_. Wait for moveit running, otherwise exit
    const std::shared_ptr<tf2_ros::Buffer> dummy_tf;
    ROS_INFO("Waiting for MoveIt servers to respond...");
    try 
    {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP_, dummy_tf, ros::Duration(50));
    } 
    catch (const std::runtime_error &e) 
    {
        ROS_ERROR("[%s] %s Continuing without MoveIt planning option.", action_name_.c_str(), e.what());
        moveit_running = false;
        return false;
    }

    // Enable visual tools
    visual_tools_ptr.reset(new moveit_visual_tools::MoveItVisualTools("base_link"));
    visual_tools_ptr->loadRemoteControl();
    ROS_INFO("[%s] Ready to receive goals.", action_name_.c_str());

    // Create action client for executing trajectory, using execute_joint_trajectory_handler
    bool create_execute_client = nh_.param("create_execute_client", false);
    if (create_execute_client)
        createArmClient(ArmClient);
    
    // Start Comau Plan Action Server
    ROS_INFO_STREAM("Starting up the Comau Plan Action Server ...  ");
    try 
    {
        as_ptr_.reset(new MoveToPoseMoveItActionServer(nh_, action_name_, boost::bind(&MoveToPoseHandlerServer::executeCallback, this, _1), false));
        as_ptr_->start();
    } 
    catch (...) {
        ROS_ERROR_STREAM("Comau Plan Action Server cannot not start.");
        return false;
    }
    ROS_INFO_STREAM("Ready to receive goals!  ");
    return true;   
}

// Create a client for communicating with execute_joint_trajectory_handler server
void MoveToPoseHandlerServer::createArmClient(arm_control_client_Ptr &actionClient) 
{
  ROS_INFO("Creating action client to arm controller ...");
  actionClient.reset(new arm_control_client("execute_joint_trajectory_handler"));
  int iterations = 0, max_iterations = 3;
  // Wait for arm controller action server to come up
  while (!actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations) {
    ROS_WARN("Waiting for the arm_trajectory_action server to come up");
    ++iterations;
  }

  if (iterations == max_iterations)
    ROS_WARN("Error in createArmClient: arm controller action server not available");
}

// Execute action body when the server is triggered (goal) by a client (client in the microservice) 
// Refer to comau_msgs/MoveToJointsMoveIt.action to change goal format
void MoveToPoseHandlerServer::executeCallback(const comau_msgs::MoveToPoseMoveItGoalConstPtr &goal)
{
    visual_tools_ptr->deleteAllMarkers();
    // Start a timer
    ros::WallTime _start;
    _start = ros::WallTime::now(); // Start timer
    ROS_INFO("[%s] MoveJoints goal received", action_name_.c_str());

    // Check if moveit framework is running. If false, abort goal request by the client
    if (!moveit_running) 
    {
        ROS_ERROR("Trying to plan goal with MoveIt but MoveIt is not running.");
        feedback_.action_feedback.millis_passed = uint((ros::WallTime::now() - _start).toNSec() * 1e-6);
        // feedback_.state = "Cannot process goal";
        as_ptr_->publishFeedback(feedback_);
        ros::Duration(0.1).sleep();
        result_.action_result.success = false;
        result_.action_result.status = comau_msgs::ActionResultStatusConstants::OPERATIONAL_EXCEPTION;
        result_.action_result.millis_passed = feedback_.action_feedback.millis_passed;
        as_ptr_->setAborted(result_);
        printLine();
        return;
    }
    
    //------------------------------------------------- PLAN ---------------------------------------------------------------------//

    
    
    if(!goal->execute)
    {
        geometry_msgs::PoseStamped transformed_pose = changePoseFrame("base_link", goal->target_pose);
        showTargetPose(transformed_pose);
        if(!validityCheck(goal->target_pose))
        {
            ROS_ERROR("Target Pose is out of robot workspace! Plan is aborted.");
            feedback_.action_feedback.millis_passed = uint((ros::WallTime::now() - _start).toNSec() * 1e-6);
            // feedback_.state = "Cannot process goal";
            as_ptr_->publishFeedback(feedback_);
            ros::Duration(0.1).sleep();
            result_.action_result.success = false;
            result_.action_result.status = comau_msgs::ActionResultStatusConstants::OPERATIONAL_EXCEPTION;
            result_.action_result.millis_passed = feedback_.action_feedback.millis_passed;
            as_ptr_->setAborted(result_);
            printLine();
            return;
        }
        // Setting up the planner
        ROS_INFO("[%s] Planning frame: %s", action_name_.c_str(), move_group_->getPlanningFrame().c_str());
        ROS_INFO("[%s] End effector link: %s", action_name_.c_str(), move_group_->getEndEffectorLink().c_str());
        std::cout << "\n";
	// Problem here with melodic?(compilation fail!)
        //move_group_->setPlanningPipelineId(goal->planning_pipeline);
        //move_group_->setPlannerId(goal->geometric_planner);
        //move_group_->setMaxVelocityScalingFactor(double(goal->velocity));
        //move_group_->setMaxAccelerationScalingFactor(double(goal->acceleration));
        //move_group_->setPlanningTime(double(goal->timeout));
        //move_group_->allowReplanning(true);
        //move_group_->setNumPlanningAttempts(1);
        
        // const std::map<std::string, std::string> params = {{"maximum_waypoint_distance","0.001"}};
        // move_group_->setPlannerParams(move_group_->getPlannerId(),PLANNING_GROUP_,params,false);                                            
                                                    
                                                    
       

        // Set Target Pose
     
        if (transformed_pose.header.frame_id == "link_6") // why?
        {
            feedback_.action_feedback.millis_passed = uint((ros::WallTime::now() - _start).toNSec() * 1e-6);
            // feedback_.state = "Cannot process goal";
            as_ptr_->publishFeedback(feedback_);
            ros::Duration(0.1).sleep(); // Wait for feedback to be updated before
                                        // publishing the result
            result_.action_result.success = false;
            result_.action_result.status = comau_msgs::ActionResultStatusConstants::OPERATIONAL_EXCEPTION;
            result_.action_result.millis_passed = feedback_.action_feedback.millis_passed;
            ROS_WARN("[%s] Set Aborted from frameID ", action_name_.c_str());
            as_ptr_->setAborted(result_);
            printLine();
            return;
        } 
        else 
        {
            // std::vector<double> joint_value_target; 
            // robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
            // robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
            // robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
            // joint_model_group = kinematic_model->getJointModelGroup(PLANNING_GROUP_); 
            // kinematic_state->setFromIK(joint_model_group, transformed_pose.pose); 
            // kinematic_state->copyJointGroupPositions(joint_model_group, joint_value_target);
            // move_group_->setJointValueTarget(joint_value_target);

            move_group_->setPoseTarget(transformed_pose);
        }

        // Plan in a dedicated thread
        plannig_thread_done_ = false;
        std::string plan_id = goal->id;
        bool hasPlan = goal->hasPlan;

        if(hasPlan)
        {
            plan_.planning_time_ = goal->plan.planning_time;
            plan_.start_state_ = goal->plan.trajectory_start;
            plan_.trajectory_ = goal->plan.trajectory;
            planning_succeeded_ = true;
            plannig_thread_done_ = true;
        }
        else
        {
            boost::thread planning_thread(&MoveToPoseHandlerServer::moveit_planning, this);        
        }
                       

        // While planning, publish feedback and check if something goes wrong, every 1ms
        while (!plannig_thread_done_) 
        {
            ros::Duration(0.001).sleep();
            feedback_.action_feedback.millis_passed = uint((ros::WallTime::now() - _start).toNSec() * 1e-6);
            // feedback_.state = "Planning...";
            as_ptr_->publishFeedback(feedback_);

            if (as_ptr_->isPreemptRequested() || !ros::ok()) // stop if ros shutdown or a new request is preempted(?)
            {
                move_group_->stop();
                if (goal->constraint_mode)
                    move_group_->clearPathConstraints();

                feedback_.action_feedback.millis_passed = uint((ros::WallTime::now() - _start).toNSec() * 1e-6);
                // feedback_.state = "Planning was Stoped";
                as_ptr_->publishFeedback(feedback_);
                ros::Duration(0.1).sleep(); // Wait for feedback to be updated before
                                            // publishing the result
                ROS_WARN("[%s] Preempted", action_name_.c_str());
                result_.action_result.success = false;
                result_.action_result.status = comau_msgs::ActionResultStatusConstants::CANCELLED;
                result_.action_result.millis_passed = feedback_.action_feedback.millis_passed;
                as_ptr_->setPreempted(result_);
                printLine();
                return;
            }
        }

        // Abort if plan does not succed
        if (!planning_succeeded_) 
        {
            if (goal->constraint_mode)
                move_group_->clearPathConstraints();
            ROS_ERROR("[%s] Planning joint goal FAILED", action_name_.c_str());
            feedback_.action_feedback.millis_passed = uint((ros::WallTime::now() - _start).toNSec() * 1e-6);
            // feedback_.state = "Planning failed";
            as_ptr_->publishFeedback(feedback_);
            ros::Duration(0.1).sleep(); // Wait for feedback to be updated before
                                        // publishing the result
            ROS_ERROR("[%s]: Aborted. No plan found.", action_name_.c_str());
            result_.action_result.success = false;
            result_.action_result.status = comau_msgs::ActionResultStatusConstants::OPERATIONAL_EXCEPTION;
            result_.action_result.millis_passed = feedback_.action_feedback.millis_passed;
            as_ptr_->setAborted(result_);
            return;
        }

        
        else
        {
            // Visualize plan
            ROS_INFO("[%s] Planning joint goal SUCCEEDED", action_name_.c_str());
            visual_tools_ptr->deleteAllMarkers();
            ROS_INFO("Visualizing plan to target: %s", planning_succeeded_ ? "SUCCEEDED" : "FAILED");
            joint_model_group = move_group_->getCurrentState()->getJointModelGroup(PLANNING_GROUP_);
            visual_tools_ptr->publishTrajectoryLine(plan_.trajectory_, joint_model_group);
            visual_tools_ptr->trigger();
            ROS_INFO_STREAM("Time planning: " << plan_.planning_time_);
            // Convert plan to ros msg and send to client
            moveit_msgs::MotionPlanResponse path;
            path.planning_time = plan_.planning_time_;
            path.group_name = PLANNING_GROUP_;
            path.trajectory_start = plan_.start_state_;
            path.trajectory = plan_.trajectory_;
            

            showPlannedPath(plan_);
            // Only in Simulation
            result_.action_result.success = true;
            result_.action_result.status = comau_msgs::ActionResultStatusConstants::SUCCESS;
            result_.action_result.millis_passed = feedback_.action_feedback.millis_passed;
            result_.action_result.plan = path;
            as_ptr_->setSucceeded(result_);

            // Test path length
            // l_ = l_ + pathLength(path);
            // ROS_INFO_STREAM("Total Length :" << l_);
            
        }
        
    }//----------------------------------------------- END PLAN ---------------------------------------------------------------//

    //----------------------------------------------  EXECUTE ---------------------------------------------------------------//
    else
    {   
        // Send trajectory to real robot using execute_joint_trajectory action client, only if specified
        if (goal->send_trajectory && validateExecution()) 
        {
            traj_goal.trajectory = plan_.trajectory_.joint_trajectory.points;  // Get the trajectory        
            ArmClient->sendGoal(traj_goal); // Send the trajectory to the execute_joint_trajectory action server          
            // TODO: test on real robot
            // execution_thread_done_ = false;
            // boost::thread execution_thread(&MoveToPoseHandlerServer::controller_execution, this);
            // while (!execution_thread_done_) 
            // {
            //     ros::Duration(0.001).sleep();
            //     feedback_.action_feedback.millis_passed = uint((ros::WallTime::now() - _start).toNSec() * 1e-6);
            //     as_ptr_->publishFeedback(feedback_);

            //     if (as_ptr_->isPreemptRequested() || !ros::ok()) // stop if ros shutdown or a new request is preempted(?)
            //     {
            //         //move_group_->stop(); // sostituire con una funzione che ferma il robot
            //         feedback_.action_feedback.millis_passed = uint((ros::WallTime::now() - _start).toNSec() * 1e-6);
            //         as_ptr_->publishFeedback(feedback_);
            //         ros::Duration(0.1).sleep(); // Wait for feedback to be updated before
            //                                     // publishing the result
            //         ROS_WARN("[%s] Preempted", action_name_.c_str());
            //         result_.action_result.success = false;
            //         result_.action_result.status = comau_msgs::ActionResultStatusConstants::CANCELLED;
            //         result_.action_result.millis_passed = feedback_.action_feedback.millis_passed;
            //         as_ptr_->setPreempted(result_);
            //         printLine();
            //         return;
            //     }
            // }
        }
        else
        {
            // Execute in Simulation
            execution_thread_done_ = false;
            boost::thread execution_thread(&MoveToPoseHandlerServer::moveit_execution, this);

            // While executing IN SIMULATION, publish feedback and check if something goes wrong, every 1ms
            while (!execution_thread_done_) 
            {
                ros::Duration(0.001).sleep();
                feedback_.action_feedback.millis_passed = uint((ros::WallTime::now() - _start).toNSec() * 1e-6);
                as_ptr_->publishFeedback(feedback_);

                if (as_ptr_->isPreemptRequested() || !ros::ok()) // stop if ros shutdown or a new request is preempted(?)
                {
                    move_group_->stop();
                    feedback_.action_feedback.millis_passed = uint((ros::WallTime::now() - _start).toNSec() * 1e-6);
                    as_ptr_->publishFeedback(feedback_);
                    ros::Duration(0.1).sleep(); // Wait for feedback to be updated before
                                                // publishing the result
                    ROS_WARN("[%s] Preempted", action_name_.c_str());
                    result_.action_result.success = false;
                    result_.action_result.status = comau_msgs::ActionResultStatusConstants::CANCELLED;
                    result_.action_result.millis_passed = feedback_.action_feedback.millis_passed;
                    as_ptr_->setPreempted(result_);
                    printLine();
                    return;
                }
            }      
        }

        // TODO : validate when executing on real robot
        if (execution_succeeded_) 
        {
            if (goal->constraint_mode)
                move_group_->clearPathConstraints();
            ROS_INFO("[%s] Succeeded", action_name_.c_str());
            // feedback_.state = "Trajectory execution completed";
            feedback_.action_feedback.millis_passed = uint((ros::WallTime::now() - _start).toNSec() * 1e-6);
            as_ptr_->publishFeedback(feedback_);
            ros::Duration(0.1).sleep(); // Wait for feedback to be updated before
                                        // publishing the result
            result_.action_result.success = true;
            result_.action_result.status = comau_msgs::ActionResultStatusConstants::SUCCESS;
            result_.action_result.millis_passed = feedback_.action_feedback.millis_passed;
            as_ptr_->setSucceeded(result_);
        } 
        else 
        {
            if (goal->constraint_mode)
            {
                move_group_->clearPathConstraints();
            }
                
            // TODO: Sepcify any exceptions
            // feedback_.state = "Trajectory execution failed";
            feedback_.action_feedback.millis_passed = uint((ros::WallTime::now() - _start).toNSec() * 1e-6);
            as_ptr_->publishFeedback(feedback_);
            ros::Duration(0.1).sleep(); // Wait for feedback to be updated before publishing the result                                     
            result_.action_result.success = false;
            result_.action_result.status = comau_msgs::ActionResultStatusConstants::OPERATIONAL_EXCEPTION;
            result_.action_result.millis_passed = feedback_.action_feedback.millis_passed;
            as_ptr_->setAborted(result_);
        }
        ////////////////////////////////////////------------ END EXECUTE --------------------------------- //////////////////////////////////////////
    }
    printLine();
}

// Plan on a separeted thread
void MoveToPoseHandlerServer::moveit_planning()
{
    // Test load plan
    //plan_ = loadPlan();
    //planning_succeeded_ = true;
    move_group_->setStartStateToCurrentState();
    planning_succeeded_ = (move_group_->plan(plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    plannig_thread_done_ = true;  
}

// Execute in Simulation
void MoveToPoseHandlerServer::moveit_execution() 
{
    if(validateExecution())
    {
        execution_succeeded_ = (move_group_->execute(plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);       
    }
    else
    {
        execution_succeeded_ = false;
    }
    execution_thread_done_ = true;  
}

void MoveToPoseHandlerServer::controller_execution()
{
    // TODO: Test on real robot
    traj_goal.trajectory = plan_.trajectory_.joint_trajectory.points;  // Get the trajectory        
    ArmClient->sendGoal(traj_goal); // Send the trajectory to the execute_joint_trajectory action server
    bool finished_before_timeout = ArmClient->waitForResult(ros::Duration(50)); // aumentare questo timeout!
    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ArmClient->getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
        if(state.toString() == "SUCCEEDED")
        {
            execution_succeeded_ = true;
        }
        else
        {
            execution_succeeded_ = false;
        }
    }
    else
    {
        ROS_INFO("Action did not finish before the time out.");
        execution_succeeded_ = false;
    }             
    execution_thread_done_ = true;
}

geometry_msgs::PoseStamped MoveToPoseHandlerServer::changePoseFrame(const std::string &target_frame,const geometry_msgs::PoseStamped &goal_pose) 
{                                                                   
    tf2_ros::Buffer br;
    br.setUsingDedicatedThread(true);
    tf2_ros::TransformListener tf2_listener(br);
    geometry_msgs::TransformStamped transform;
    geometry_msgs::PoseStamped transformed_pose;
    try 
    {
        transform = br.lookupTransform(target_frame, goal_pose.header.frame_id, ros::Time(0), ros::Duration(1.0));
        tf2::doTransform(goal_pose, transformed_pose, transform);
        ROS_INFO_STREAM("Change transform pose to..   " << transformed_pose);
        return transformed_pose;
    } 
    catch (tf2::LookupException &e) 
    {
        ROS_ERROR("[%s] %s", action_name_.c_str(), e.what());
        transformed_pose.header.frame_id = "link_6";
        return transformed_pose;
    }
}

void MoveToPoseHandlerServer::printLine() 
{
  struct winsize size;
  ioctl(STDOUT_FILENO, TIOCGWINSZ, &size);
  for (uint i = 0; i < size.ws_col; i++)
    std::cout << "=";
  std::cout << "\n";
}

bool MoveToPoseHandlerServer::validityCheck(geometry_msgs::PoseStamped target)
{
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    // Check if target pose is reachable
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    joint_model_group = kinematic_model->getJointModelGroup(PLANNING_GROUP_);

    //return kinematic_state->setFromIK(joint_model_group,target.pose,1.0);
    return true;
}

void MoveToPoseHandlerServer::showTargetPose(geometry_msgs::PoseStamped target)
{
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();                     
    text_pose.translation().z() = 1.5;
    std::string text = "Comau Plan Demo";
    visual_tools_ptr->publishText(text_pose, "Comau Plan", rviz_visual_tools::WHITE, rviz_visual_tools::XXXLARGE);
    std::string pose_label;
    pose_label = "Target Pose ";
    visual_tools_ptr->publishAxisLabeled(target.pose, pose_label,rviz_visual_tools::SMALL);
    visual_tools_ptr->trigger();
} 

void MoveToPoseHandlerServer::showPlannedPath(moveit::planning_interface::MoveGroupInterface::Plan plan_) 
{
    visual_tools_ptr->publishTrajectoryLine(plan_.trajectory_, joint_model_group->getLinkModel("ee_link"),joint_model_group);  
    visual_tools_ptr->trigger();
}

void MoveToPoseHandlerServer::savePlan(moveit::planning_interface::MoveGroupInterface::Plan plan, std::string file_path, std::string plan_id)
{
    ROS_INFO_STREAM("Saving path : ID - " << plan_id << " || File path : " << file_path);
    moveit_msgs::MotionPlanResponse path;
    path.planning_time = plan.planning_time_;
    path.group_name = PLANNING_GROUP_;
    path.trajectory_start = plan.start_state_;
    path.trajectory = plan.trajectory_;
    rosbag::Bag bag;
    bag.open(file_path + "/" + plan_id + ".bag", rosbag::bagmode::Write); 
    bag.write(plan_id, ros::Time::now(), path);
    bag.close();  
    ROS_INFO("Path saved...");
}

moveit::planning_interface::MoveGroupInterface::Plan MoveToPoseHandlerServer::loadPlan(std::string file_path,std::string plan_id)
{
    ROS_INFO_STREAM("Loading plan from " << file_path << "with ID" << plan_id);
    rosbag::Bag bag;
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    //bag.open(file_path, rosbag::bagmode::Read); // file path completo
    bag.open(file_path + "/" + plan_id + ".bag", rosbag::bagmode::Read); // file path completo
    std::vector<std::string> topics;
    //topics.push_back(plan_id);
    topics.push_back(plan_id);
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    foreach(rosbag::MessageInstance const m, view)
    {
        moveit_msgs::MotionPlanResponseConstPtr s = m.instantiate<moveit_msgs::MotionPlanResponse>();
        if(s == nullptr)
        {
            ROS_INFO("s in nullptr");
        }
        plan.planning_time_ = s->planning_time;
        plan.start_state_ = s->trajectory_start;
        plan.trajectory_ = s->trajectory;
    }
    bag.close();
    return plan;
}

Eigen::Isometry3d MoveToPoseHandlerServer::fk(std::vector<double> joint_values)
{
    
    //moveit::planning_interface::MoveGroupInterface move_group_interface_("arm");
    //const moveit::core::JointModelGroup* joint_model_group = move_group_interface_.getCurrentState()->getJointModelGroup("arm");
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));   
    joint_model_group = kinematic_model->getJointModelGroup("arm");
    kinematic_state->setJointGroupPositions(joint_model_group,joint_values);
    return kinematic_state->getGlobalLinkTransform("ee_link");
};

double MoveToPoseHandlerServer::pathLength(moveit_msgs::MotionPlanResponse plan)
{
    int num_poses = plan.trajectory.joint_trajectory.points.size();
    Eigen::Isometry3d p1,p2;
    double len = 0;
    double l;

    for (int i = 0; i < num_poses-1; i++)
    {
        p1 = fk(plan.trajectory.joint_trajectory.points[i].positions);
        p2 = fk(plan.trajectory.joint_trajectory.points[i+1].positions); 
        l = pow(pow((p2.translation().x() - p1.translation().x()),2) +
                pow((p2.translation().y() - p1.translation().y()),2) +
                pow((p2.translation().z() - p1.translation().z()),2),0.5);
        ROS_INFO_STREAM("Distance: " << l);
        len = len + l;
    }
    return len;
};

bool MoveToPoseHandlerServer::validateExecution()
{
    moveit::core::RobotStatePtr current_state = move_group_->getCurrentState();
    sensor_msgs::JointState current_joint_state;
    moveit::core::robotStateToJointStateMsg(*current_state,current_joint_state);
    for (size_t i = 0; i < current_joint_state.position.size(); i++)
    {
        if((current_joint_state.position[i] - plan_.start_state_.joint_state.position[i]) > eps_)
        {
            ROS_ERROR("Current state differs from planned start state. Unable to execute planned path.");
            return false;
        }
    }
    return true;
}

