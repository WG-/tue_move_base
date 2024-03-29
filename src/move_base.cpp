#include <move_base/move_base.h>
#include <visualization_msgs/Marker.h>
#include <boost/algorithm/string.hpp>

namespace move_base {

MoveBase::MoveBase(std::string name, tf::TransformListener& tf) :
    tf_(tf),
    as_(NULL),
    //local_planner_(NULL),     // deprecated as this is handled by the shared pointer from the boost library
    //global_planner_(NULL),    // deprecated as this is handled by the shared pointer from the boost library
    bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"),
    blp_loader_("nav_core", "nav_core::BaseLocalPlanner")
{

    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;

    //as_ = new MoveBaseActionServer(ros::NodeHandle(), "move_base", boost::bind(&MoveBase::executeCb, this, _1), false);

    // Construct action server
    as_ = new  MoveBaseActionServer(nh, "move_base", false);
    as_->registerGoalCallback(boost::bind(&MoveBase::goalCallback, this));
    as_->registerPreemptCallback(boost::bind(&MoveBase::preemtCallback, this));

    //get some parameters that will be global to the move base node
    std::string global_planner, local_planner;
    private_nh.param("base_global_planner", global_planner, std::string("navfn/NavfnROS"));
    private_nh.param("base_local_planner", local_planner, std::string("base_local_planner/TrajectoryPlannerROS"));
    private_nh.param("global_costmap_laser/robot_base_frame", robot_base_frame_, std::string("base_link"));;
    private_nh.param("global_costmap_laser/global_frame", global_frame_, std::string("/map"));
    double time_to_replan_sec;
    private_nh.param("time_to_replan", time_to_replan_sec, 10.0);
    time_to_replan_ = ros::Duration(time_to_replan_sec);
    private_nh.param("max_time_to_execute_replan", max_time_to_replan_, 30.0);
    private_nh.param("goal_area_radius", goal_area_radius_, 0.1);
    private_nh.param("replan_length_difference_factor", replan_length_diff_factor_, 1.5);
    private_nh.param("replan_length_difference_tolerance", replan_length_diff_tol_, 0.2);

    // from now on we will only use the global costmap
    // pub_local_costmap_ = private_nh.advertise<visualization_msgs::Marker>("local_costmap", 2);
    pub_global_costmap_ = private_nh.advertise<visualization_msgs::Marker>("global_costmap", 2);

    //for commanding the base
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // markers for debugging
    debug_marker_pub_ = private_nh.advertise<visualization_msgs::Marker>("debug_marker", 1 );

    /// for visual feedback
    goal_pose_pub_ = private_nh.advertise<visualization_msgs::Marker>("goal_pose", 1);
    current_plan_pub_ = private_nh.advertise<visualization_msgs::Marker>("current_path", 1);
    re_plan_pub_ = private_nh.advertise<visualization_msgs::Marker>("replanned_path", 1);

    /// action server
    ros::NodeHandle action_nh("move_base");
    action_goal_pub_ = action_nh.advertise<tue_move_base_msgs::MoveBaseActionGoal>("goal", 1);

    //we'll provide a mechanism for some people to send goals as PoseStamped messages over a topic
    //they won't get any useful information back about its status, but this is useful for tools
    //like nav_view and rviz
    ros::NodeHandle simple_nh("move_base_simple");
    goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&MoveBase::simpleCallback, this, _1));

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     *                     	GLOBAL COSTMAP
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

    global_costmap_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
    global_costmap_->pause();

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     *                     	LOCAL COSTMAP
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

    // from now on we will only use the global costmap
    //local_costmap_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
    //local_costmap_->pause();

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     *                     	STATIC COSTMAP
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

    static_costmap_ = new costmap_2d::Costmap2DROS("static_costmap", tf_);
    static_costmap_->pause();

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     *                     	GLOBAL PLANNER
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

    //initialize the global planner
    try {
        //check if a non fully qualified name has potentially been passed in
        if(!bgp_loader_.isClassAvailable(global_planner)){
            std::vector<std::string> classes = bgp_loader_.getDeclaredClasses();
            for(unsigned int i = 0; i < classes.size(); ++i){
                if(global_planner == bgp_loader_.getName(classes[i])){
                    //if we've found a match... we'll get the fully qualified name and break out of the loop
                    ROS_WARN("Planner specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                             global_planner.c_str(), classes[i].c_str());
                    global_planner = classes[i];
                    break;
                }
            }
        }

        // global_planner_ = bgp_loader_.createUnmanagedInstance(global_planner);
        // changed from createClassInstance (deprecated) to createInstance
        global_planner_ = bgp_loader_.createInstance(global_planner);
        global_planner_->initialize(bgp_loader_.getName(global_planner), global_costmap_);
		ROS_INFO("Initialized global planner");
    } catch (const pluginlib::PluginlibException& ex)
    {
        ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
        exit(0);
    }

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     *                     	LOCAL PLANNER
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

    //create a local planner
    try {
        //check if a non fully qualified name has potentially been passed in
        if(!blp_loader_.isClassAvailable(local_planner)){
            std::vector<std::string> classes = blp_loader_.getDeclaredClasses();
            for(unsigned int i = 0; i < classes.size(); ++i){
                if(local_planner == blp_loader_.getName(classes[i])){
                    //if we've found a match... we'll get the fully qualified name and break out of the loop
                    ROS_WARN("Planner specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                             local_planner.c_str(), classes[i].c_str());
                    local_planner = classes[i];
                    break;
                }
            }
        }

        // local_planner_ = blp_loader_.createUnmanagedInstance(local_planner);
        // changed from createClassInstance (deprecated) to createInstance
        local_planner_ = blp_loader_.createInstance(local_planner);
        // also init the local planner with the global costmap
        local_planner_->initialize(blp_loader_.getName(local_planner), &tf_, global_costmap_);
		ROS_INFO("Initialized local planner");
    } catch (const pluginlib::PluginlibException& ex)
    {
        ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", local_planner.c_str(), ex.what());
        exit(0);
    }

    // Start actively updating costmaps based on sensor data
    // local_costmap_->start();
    global_costmap_->start();
    static_costmap_->start();

    //advertise a service for getting a plan
    make_plan_srv_ = private_nh.advertiseService("get_plan", &MoveBase::planService, this);

    query_costmap_srv_ = private_nh.advertiseService("query_costmap", &MoveBase::queryCostmapPointService, this);

    clear_costmap_srv_ = private_nh.advertiseService("clear_costmap", &MoveBase::clearCostmapService, this);

    //we're all set up now so we can start the action server
    as_->start();
	ROS_INFO("Action server started, ready to go!");
}

MoveBase::~MoveBase(){

    if (as_) {
        delete as_;
    }

    //if (global_planner_) {
    //    delete global_planner_;   // deprecated as class loader lib now uses shared pointers --> deletion is handled by boost library
    //}

    //if (local_planner_) {
    //    delete local_planner_;    // deprecated as class loader lib now uses shared pointers --> deletion is handled by boost library
    //}

    delete global_costmap_;
    //delete local_costmap_;
}

/**********************************************************
 *                      CALLBACKS
 **********************************************************/

void MoveBase::simpleCallback(const geometry_msgs::PoseStamped::ConstPtr& goal) {
    ROS_DEBUG_NAMED("move_base","In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");

    geometry_msgs::PoseStamped robot_pose_msg;
    robot_pose_msg.header.frame_id = global_frame_;
    robot_pose_msg.header.stamp = ros::Time();

    global_costmap_->getRobotPose(robot_pose_);
    tf::poseStampedTFToMsg(robot_pose_, robot_pose_msg);

    tue_move_base_msgs::MoveBaseActionGoal action_goal;
    action_goal.header.stamp = ros::Time::now();

    t_last_cmd_vel_ = ros::Time::now();
    t_last_replan_ = ros::Time::now();
    path_time_out_ = ros::Duration(0);
    prev_path_length_factor_ = 0.0;
    replanned_path_.clear();

    if (global_planner_->makePlan(robot_pose_msg, *goal, action_goal.goal.path)) {
        action_goal_pub_.publish(action_goal);
    }
    // check if there is a feasible goal pose within the goal_area_radius
    else {
        geometry_msgs::PoseStamped new_goal = *goal;
        updateGoal(new_goal);
        ROS_INFO("[tue_move_base] updated goal pose");
        if(global_planner_->makePlan(robot_pose_msg, new_goal, action_goal.goal.path))
            action_goal_pub_.publish(action_goal);
    }
}

void MoveBase::goalCallback() {
    tue_move_base_msgs::MoveBaseGoal goal = *as_->acceptNewGoal();
    current_path_ = goal.path;
    local_planner_->setPlan(current_path_);
    current_goal_ = goal.path.back();
    t_last_cmd_vel_ = ros::Time::now();
    t_last_replan_ = ros::Time::now();
    path_time_out_ = ros::Duration(0);
    prev_path_length_factor_ = 0.0;
    replanned_path_.clear();
}

void MoveBase::preemtCallback() {
    as_->setPreempted();
}

bool MoveBase::queryCostmapPointService(tue_costmap_msgs::PointQuery::Request& req, tue_costmap_msgs::PointQuery::Response& resp) {

    costmap_2d::Costmap2D global_costmap_copy;
    global_costmap_->getCostmapCopy(global_costmap_copy);

    costmap_2d::Costmap2D static_costmap_copy;
    static_costmap_->getCostmapCopy(static_costmap_copy);

    for(vector<geometry_msgs::PointStamped>::const_iterator it_point = req.points.begin(); it_point != req.points.end(); ++it_point) {
        tf::Stamped<tf::Point> point;
        tf::pointStampedMsgToTF(*it_point, point);
        point.stamp_ = ros::Time();

        tf::Stamped<tf::Point> point_GLOBAL;
        tf_.transformPoint(global_costmap_->getGlobalFrameID(), point, point_GLOBAL);

        unsigned int mx, my;
        unsigned char cost;

        // set the cost to -1 if the transform is out of bounds of the map
        if (global_costmap_copy.worldToMap(point_GLOBAL.getX(), point_GLOBAL.getY(), mx, my)) {}
        else {
            cost = -1;
        }
        cost = max(global_costmap_copy.getCost(mx, my), static_costmap_copy.getCost(mx, my));

        tue_costmap_msgs::PointInfo point_info;
        point_info.point = *it_point;

        // todo: better distance representation
        point_info.distance_to_closest_obstacle = 1 - (double)cost / 255;
        point_info.cost = (double)cost;
        point_info.resolution = global_costmap_copy.getResolution();
        point_info.size_cells_x = global_costmap_copy.getSizeInCellsX();
        point_info.size_cells_y = global_costmap_copy.getSizeInCellsY();
        resp.points_info.push_back(point_info);
    }

    return true;

}

bool MoveBase::clearCostmapService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {

    ROS_INFO("[tue_move_base] clearing costmap");

    global_costmap_->resetMapOutsideWindow(0,0);

    ROS_INFO("[tue_move_base] cleared costmap");

    return true;
}

bool MoveBase::planService(tue_move_base_msgs::GetPath::Request &req, tue_move_base_msgs::GetPath::Response &res) {

    ROS_INFO("[tue_move_base] planService");

    geometry_msgs::PoseStamped robot_pose_msg;
    robot_pose_msg.header.frame_id = global_frame_;
    robot_pose_msg.header.stamp = ros::Time();

    global_costmap_->getRobotPose(robot_pose_);
    tf::poseStampedTFToMsg(robot_pose_, robot_pose_msg);

    req.target_pose.header.stamp = ros::Time();

    goal_area_radius_ = req.goal_area_radius;

    global_planner_->makePlan(robot_pose_msg, req.target_pose, res.path);

    // check if there is a feasible goal pose within the goal_area_radius
    if (res.path.empty()) {
        updateGoal(req.target_pose);
        ROS_INFO("[tue_move_base] updated goal pose");
        global_planner_->makePlan(robot_pose_msg, req.target_pose, res.path);
    }

    return true;
}

/**********************************************************
 *                  GENERATE REFERENCE
 **********************************************************/


void MoveBase::generateReference() {

    // clear the local and the global costmap
    // so only the static map + the latest sensor readings are in the global costmap
    // and only the latest sensor readings are in the local costmap
    //local_costmap_->resetMapOutsideWindow(0,0); --> from now on we will only use the global costmap

    // NOTE BAS: as we now use 2 laser scanner the costmap should NOT be cleared everytime because then
    // readings of the front and back laser are erased every iteration.
    //global_costmap_->resetMapOutsideWindow(0,0);

    // publishCostmap(*local_costmap_, 200, pub_local_costmap_);
    publishCostmap(*global_costmap_, 200, pub_global_costmap_);
    publishGoal(current_goal_, goal_pose_pub_);

    // declare the feedback message
    tue_move_base_msgs::MoveBaseFeedback feedback_msg;

    // the number of poses left on the current path
    int nr_poses_to_goal = -1;

    // do not continue if the action client has been aborted
    if (!as_->isActive()) {
        return;
    }
    // notify the actien client if the goal is reached
    // and return, otherwise an empty plan is fed to the local planner!
    if (local_planner_->isGoalReached()) {
		/// make the robot stand still
        geometry_msgs::Twist cmd_vel;
        cmd_vel.angular.x = 0;
        cmd_vel.angular.y = 0;
        cmd_vel.angular.z = 0;
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel.linear.z = 0;
        vel_pub_.publish(cmd_vel);
        as_->setSucceeded();
        ROS_INFO("[tue_move_base] goal reached --> SUCCEED action client");
        return;
    }

    // the current waypoint on the path
    current_waypoint_ = determineCurrentWaypoint();

    // determine number of poses to the goal
    // if there is no path and no current waypoint it will equal -1
    nr_poses_to_goal = current_path_.size() - abs(current_waypoint_);

    geometry_msgs::Twist cmd_vel;
    geometry_msgs::PointStamped obstacle_position;
    // init to -1 so executive can distinguish the noticed obstacle in the feedback message
    obstacle_position.point.x = -1;
    obstacle_position.point.y = -1;
    obstacle_position.point.z = -1;
    obstacle_position.header.frame_id = global_frame_;

    bool goal_blocked = false;

    bool local_plan_free = local_planner_->computeVelocityCommands(cmd_vel);
    bool global_plan_free = checkGlobalPath(obstacle_position, goal_blocked);

    // generate a feedback message with the current base position,
    // a possible obstacle position and the number of poses left to the goal
    global_costmap_->getRobotPose(robot_pose_);
    // directly transform the robot pose to the feedback message
    tf::poseStampedTFToMsg(robot_pose_, feedback_msg.base_position);
    feedback_msg.obstacle_position = obstacle_position;
    feedback_msg.nr_poses_to_goal = nr_poses_to_goal;

    ROS_DEBUG_STREAM("obstacle at x = " << obstacle_position.point.x << " y = " << obstacle_position.point.y <<
                     "   nr_poses_to_goal: " << nr_poses_to_goal);

    /// if the global plan is free and not empty (otherwise it is also free!),
    /// check whether the local plan is free or not
    if (global_plan_free && !current_path_.empty()){
        // if the local plan is also free, publish the command
        if (local_plan_free){
            vel_pub_.publish(cmd_vel);
            t_last_cmd_vel_ = ros::Time::now();
            //ROS_INFO("Case 1: global free + local free");
        }
        /// if the local plan is not free, something strange is going on so try to re-plan
        else {
            ROS_INFO_STREAM_THROTTLE(2,"[tue_move_base] time since last cmd_vel is " << (ros::Time::now() - t_last_cmd_vel_) <<
                                     ", time to abort is " << (time_to_replan_ + path_time_out_));
            if ((ros::Time::now() - t_last_cmd_vel_) > time_to_replan_) {
                as_->setAborted();
                ROS_INFO("[tue_move_base] time to replan (%f sec.) exceeded --> ABORT action client", time_to_replan_.toSec());
            }
            else {
                ROS_DEBUG_THROTTLE(1,"[tue_move_base] global plan free, local plan blocked --> trying to re-plan");
                geometry_msgs::PoseStamped robot_pose_msg;
                robot_pose_msg.header.frame_id = global_frame_;
                robot_pose_msg.header.stamp = ros::Time();
                global_costmap_->getRobotPose(robot_pose_);
                tf::poseStampedTFToMsg(robot_pose_, robot_pose_msg);
                // do not clear current path, because it still needs to be checked whether it becomes free!
                replanned_path_.clear();
                // if a new plan can be found, send it to the local planner
                if (global_planner_->makePlan(robot_pose_msg, current_goal_, replanned_path_)) {
                    ROS_DEBUG("[tue_move_base] found a re-plan");
                    current_path_ = replanned_path_;
                    local_planner_->setPlan(current_path_);
                    replanned_path_.clear();
                }
            }
            ROS_DEBUG("Case 2: global free + local blocked ");
        }
    }
    /// if the global plan is not free, check the local path
    /// and whether an obstacle is blocking or the goal is not feasible anymore
    else {
        /// if the local plan is still free, keep going...
        /// whether the goal is infeasible or an obstacle is blocking
        if (local_plan_free) {
            if (goal_blocked){
                vel_pub_.publish(cmd_vel);
                t_last_cmd_vel_ = ros::Time::now();
                ROS_DEBUG("Case 3: global blocked + local free ");

            }
            else {
                vel_pub_.publish(cmd_vel);
                t_last_cmd_vel_ = ros::Time::now();
                ROS_DEBUG("Case 4: global blocked + local free ");
            }
        }
        /// if the local plan is not free, a new feasible goal must be computed
        /// or a re-plan is necessary
        else {
            /// try to find a new goal if the goal is blocked
            if (goal_blocked){
                updateGoal(current_goal_);
                ROS_INFO("[tue_move_base] updated goal pose");
                geometry_msgs::PoseStamped robot_pose_msg;
                robot_pose_msg.header.frame_id = global_frame_;
                robot_pose_msg.header.stamp = ros::Time();
                global_costmap_->getRobotPose(robot_pose_);
                tf::poseStampedTFToMsg(robot_pose_, robot_pose_msg);
                //if (global_planner_->makePlan(robot_pose_msg, current_goal_,current_path_))
                //    local_planner_->setPlan(current_path_);
                ROS_INFO("Case 5: global blocked + local blocked ");
            }

            /// try to re-plan for a fixed number of seconds
            /// if no new plan can be found, abort the action client
            ROS_DEBUG("Case 6: global blocked + local blocked ");
            ROS_INFO_STREAM_THROTTLE(2,"[tue_move_base] time since last cmd_vel is " << (ros::Time::now() - t_last_cmd_vel_) <<
                                     ", time to abort is " << (time_to_replan_ + path_time_out_));

            if ((ros::Time::now() - t_last_cmd_vel_) > time_to_replan_ + path_time_out_) {
                as_->setAborted();
                ROS_INFO("[tue_move_base] time to replan (%f sec.) exceeded --> ABORT action client", (time_to_replan_ + path_time_out_).toSec());
            }
            else {
                ROS_DEBUG_THROTTLE(1,"[tue_move_base] global plan blocked, local plan blocked --> trying to re-plan");
                geometry_msgs::PoseStamped robot_pose_msg;
                robot_pose_msg.header.frame_id = global_frame_;
                robot_pose_msg.header.stamp = ros::Time();
                global_costmap_->getRobotPose(robot_pose_);
                tf::poseStampedTFToMsg(robot_pose_, robot_pose_msg);
                /// do not clear current path, because it still needs to be checked whether it becomes free!
                replanned_path_.clear();

                /// if a new plan can be found, send it to the local planner
                if (global_planner_->makePlan(robot_pose_msg, current_goal_, replanned_path_)) {

                    path_length_factor_ = (double)replanned_path_.size() / (double)(current_path_.size() - current_waypoint_);
                    double path_time_out_sec = path_length_factor_ * time_to_replan_.toSec();
                    path_time_out_ = ros::Duration(std::min(path_time_out_sec, max_time_to_replan_));

                    /// check if the found re-plan is significantly different from the previous found plan
                    if (std::abs(path_length_factor_ - prev_path_length_factor_) > replan_length_diff_tol_) {
                        t_last_replan_ = ros::Time::now();
                        t_last_cmd_vel_ = ros::Time::now();
                        ROS_INFO_STREAM("[tue_move_base] found re-plan with length = " << replanned_path_.size() <<
                                        ", current plan length = " << current_path_.size() <<
                                        " - " << current_waypoint_ << " = " << (current_path_.size() - current_waypoint_));
                    }

                    ROS_DEBUG_STREAM("[tue_move_base] time since last found re-plan is " << (ros::Time::now() - t_last_replan_) <<
                                    ", time out before this re-plan is executed is " << (path_time_out_));

                    /// if the replanned path is less than a specified factor as long as the original path execute it immediately
                    if (path_length_factor_ < replan_length_diff_factor_) {
                        current_path_ = replanned_path_;
                        local_planner_->setPlan(current_path_);
                        path_time_out_ = ros::Duration(0);
                        prev_path_length_factor_ = 0.0;
                        ROS_INFO("[tue_move_base] executing re-plan immediately");
                        replanned_path_.clear();
                    }
                    /// else execute the path if the time out is exceeded
                    else if ((ros::Time::now() - t_last_replan_) > path_time_out_) {
                        current_path_ = replanned_path_;
                        local_planner_->setPlan(current_path_);
                        path_time_out_ = ros::Duration(0);
                        prev_path_length_factor_ = 0.0;
                        ROS_INFO("[tue_move_base] executing re-plan after re-plan time out");
                        replanned_path_.clear();
                    }
                    prev_path_length_factor_ = path_length_factor_;
                }
            }
        }
    }

    if (path_time_out_ > ros::Duration(0.0)) {
        feedback_msg.nr_sec_till_replan_execution = path_time_out_.toSec() - (ros::Time::now() - t_last_replan_).toSec();
        ROS_INFO_STREAM_THROTTLE(1,"time out till execution of replan = " << path_time_out_.toSec() - (ros::Time::now() - t_last_replan_).toSec() << " sec.");
    }
    else {
        feedback_msg.nr_sec_till_replan_execution = path_time_out_.toSec();
    }
    feedback_msg.path = current_path_;

    /// publish the feedback message
    as_->publishFeedback(feedback_msg);
    /// publish the plan that is being executed
    publishCurrentPlan(current_path_,0,1,0,1);
    /// publish the re-plan that has been found
    double plan_color = 0;
    if (path_time_out_ > ros::Duration(0.0))
        plan_color = (ros::Time::now()-t_last_replan_).toSec()/path_time_out_.toSec();
    publishRePlan(replanned_path_,1-plan_color,plan_color,0,1);
}

// determine the index of the waypoint on the global path closest to the robot
int MoveBase::determineCurrentWaypoint(){

    global_costmap_->getRobotPose(robot_pose_);

    double prev_dist_sq = numeric_limits<double>::max();
    int current_waypoint_index = -1;

    for(vector<geometry_msgs::PoseStamped>::iterator it = current_path_.begin(); it != current_path_.end(); ++it) {
        tf::Stamped<tf::Pose> waypoint;
        tf::poseStampedMsgToTF(*it, waypoint);
        double dist_sq = (waypoint.getOrigin() - robot_pose_.getOrigin()).length2();

        if (dist_sq < prev_dist_sq) {
            current_waypoint_index = it - current_path_.begin();
            prev_dist_sq = dist_sq;
        }
    }
    return current_waypoint_index;
}

bool MoveBase::checkGlobalPath(geometry_msgs::PointStamped& obstacle_position, bool &goal_blocked){

    // get a copy of the costmap check the costs of the path
    costmap_2d::Costmap2D costmap;
    global_costmap_->getCostmapCopy(costmap);

    tf::Stamped<tf::Pose> goal_pose;
    tf::poseStampedMsgToTF(current_goal_, goal_pose);
    goal_pose.stamp_ = ros::Time();

    // loop over the path beginning from the current waypoint because we don't want to check the part
    // that is already traversed
    for(unsigned int it_path = current_waypoint_; it_path < current_path_.size(); ++it_path) {

        tf::Stamped<tf::Pose> it_point;
        tf::poseStampedMsgToTF(current_path_[it_path], it_point);
        it_point.stamp_ = ros::Time();

        tf::Stamped<tf::Pose> pose_GLOBAL;
        tf_.transformPose(global_costmap_->getGlobalFrameID(), it_point, pose_GLOBAL);

        unsigned int mx, my;
        costmap.worldToMap(pose_GLOBAL.getOrigin().getX(), pose_GLOBAL.getOrigin().getY(), mx, my);

        unsigned char cost = costmap.getCost(mx, my);

        if (cost == costmap_2d::LETHAL_OBSTACLE || cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE){

            geometry_msgs::Point p;
            p.x = pose_GLOBAL.getOrigin().getX();
            p.y = pose_GLOBAL.getOrigin().getY();
            p.z = 0;

            //ROS_INFO("OBSTACLE encountered at x = %f and y = %f", p.x, p.y);
            obstacle_position.point.x = pose_GLOBAL.getOrigin().getX();
            obstacle_position.point.y = pose_GLOBAL.getOrigin().getY();

            // TODO: this functionality must be replaced by a seperate node
            // move_base should not determine what the new desired goal pose is
            // instead the executive in combination with seperate nodes
            // has to determine what the best pose is
            if ((goal_pose.getOrigin() - it_point.getOrigin()).length() <  goal_area_radius_)
            {
                ROS_DEBUG("pose within goal area radius");
                goal_blocked = true;
                return false;
            }

            goal_blocked = false;

            std_msgs::ColorRGBA color;
            color.r = 0;
            color.g = 1;
            color.b = 1;
            color.a = 1;
            publishDebugMarker(p,color,debug_marker_pub_);

            return false;
        }
    }
    return true;
}

bool MoveBase::updateGoal(geometry_msgs::PoseStamped& goal) {

    costmap_2d::Costmap2D costmap;
    global_costmap_->getCostmapCopy(costmap);

    int width = costmap.getSizeInCellsX();
    int height = costmap.getSizeInCellsY();
    double res =  costmap.getResolution();
    int cell_radius = max(0.0, ceil(goal_area_radius_ / res));
    double prev_cost = costmap_2d::LETHAL_OBSTACLE;
    float dist = 0.0;
    float prev_dist = 0.0;
    int new_goal_mx = 0;
    int new_goal_my = 0;

    tf::Stamped<tf::Pose> goal_pose;
    tf::poseStampedMsgToTF(goal, goal_pose);
    goal_pose.stamp_ = ros::Time();

    tf::Stamped<tf::Pose> goal_pose_GLOBAL;
    tf_.transformPose(global_costmap_->getGlobalFrameID(), goal_pose, goal_pose_GLOBAL);

    unsigned int mx_center, my_center;
    costmap.worldToMap(goal_pose_GLOBAL.getOrigin().getX(), goal_pose_GLOBAL.getOrigin().getY(), mx_center, my_center);

    // loop over the cells that are within the goal_area_radius
    for(int mx = max((int)mx_center - cell_radius, 0); mx <= min((int)mx_center + cell_radius, width - 1); ++mx) {
        for(int my = max((int)my_center - cell_radius, 0); my <= min((int)my_center + cell_radius, height - 1); ++my) {
            ROS_DEBUG_STREAM("[mx,my] = [" << mx << "," << my << "], distance to center = " << sqrt((mx_center-mx)*(mx_center-mx) + (my_center-my)*(my_center-my))*res);
            dist = sqrt((mx_center-mx)*(mx_center-mx) + (my_center-my)*(my_center-my))*res;
            // we want the new goal to be halfway between the max radius and the current goal
            // so only query the cost for cells that are within a circle with half the radius
            if (dist < goal_area_radius_)
            {
                double cost = (double)costmap.getCost((int)mx, (int)my);
                ROS_DEBUG_STREAM("cost at [" << mx << "," << my << "] = " << cost);
                // the cost should be as low as possible, which means that the cell is furthest away
                if (cost < costmap_2d::INSCRIBED_INFLATED_OBSTACLE && cost <= prev_cost)
                {
                    // the cells with the greatest distance is chosen for cells that have an equal cost
                    if (cost == prev_cost)
                    {
                        if (dist < prev_dist)
                        {
                            new_goal_mx = mx;
                            new_goal_my = my;
                            prev_cost = cost;
                            prev_dist = dist;
                        }
                    }
                    else if (cost < prev_cost){
                        new_goal_mx = mx;
                        new_goal_my = my;
                        prev_cost = cost;
                        prev_dist = dist;
                    }
                }
            }
        }
    }
    if (prev_cost > 0.0) {
        costmap.mapToWorld(new_goal_mx,new_goal_my,goal.pose.position.x,goal.pose.position.y);
        return true;
    }
    else {
        return false;
    }
}

/**********************************************************
 *                     VISUALIZATION
 **********************************************************/

void MoveBase::publishCostmap(const costmap_2d::Costmap2DROS& costmap_ros, int size, ros::Publisher& publisher) {
    costmap_2d::Costmap2D costmap;
    costmap_ros.getCostmapCopy(costmap);

    unsigned int mx_center, my_center;
    tf::Stamped<tf::Pose> robot_pose;
    costmap_ros.getRobotPose(robot_pose);
    if (!costmap.worldToMap(robot_pose.getOrigin().getX(), robot_pose.getOrigin().getY(), mx_center, my_center)) {
        return;
    }

    int width = costmap.getSizeInCellsX();
    int height = costmap.getSizeInCellsY();

    visualization_msgs::Marker marker;

    marker.header.frame_id = costmap_ros.getGlobalFrameID();
    marker.header.stamp = ros::Time::now();
    marker.ns = "costmap_amigo";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = costmap.getResolution();
    marker.scale.y = costmap.getResolution();
    marker.scale.z = 0;

    // Set alpha to 1
    marker.color.a = 1;

    marker.lifetime = ros::Duration(0.0);

    double x, y;
    for(int mx = max((int)mx_center - size, 0); mx < min((int)mx_center + size, width - 1); ++mx) {
        for(int my = max((int)my_center - size, 0); my < min((int)my_center + size, height - 1); ++my) {

            double cost = (double)costmap.getCost((int)mx, (int)my);

            costmap.mapToWorld(mx, my, x, y);

            geometry_msgs::Point p;
            p.x = x;
            p.y = y;
            p.z = 0.0;

            std_msgs::ColorRGBA color;
            color.a = 0.95;

            // Costmap_2d takes a occupancy grid and transforms it to a costmap
            //
            // Occupancy grid: each cell represents the probability of occupancy.
            // Probabilities are in the range [0,100].  Unknown is -1.
            //
            // Costmap_2d: each cell has a cost (defined in cost_values.h)
            // NO_INFORMATION = 255
            // LETHAL_OBSTACLE = 254
            // INFLATED_OBSTACLE = 253
            // FREE_SPACE = 0
            //
            // Costmap_2d sets the unknown cells from the occupancy grid to 255 as
            // it only takes unsigned values. Only if the unknown_cost_value == 255
            // and track_unknown_space is true the unknown values will be mapped to
            // the NO_INFORMATION (255) cost.
            // HOWEVER: the cost will be overwritten in costmap_2d by the cost for
            // a LETHAL_OBSTACLE as all costs >= 100 are set to LETHAL_OBSTACLE
            // Bottom line: you can't distinghuish unknown space from obstacles!
            if (cost == costmap_2d::LETHAL_OBSTACLE)
            {
                color.r = 0;
                color.g = 0;
                color.b = 0;
            }
            if (cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
            {
                color.r = 0.5;
                color.g = 0;
                color.b = 0;
            }
            if (cost == costmap_2d::FREE_SPACE)
            {
                color.r = 1;
                color.g = 1;
                color.b = 1;
            }
            else if (cost > costmap_2d::FREE_SPACE && cost < costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
            {
                color.r = 1-cost/255;
                color.g = 1-cost/255;
                color.b = 1-cost/255;
            }

            marker.points.push_back(p);
            marker.colors.push_back(color);
        }
    }

    publisher.publish(marker);
}

void MoveBase::publishGoal(geometry_msgs::PoseStamped& goal, ros::Publisher& publisher) {

    visualization_msgs::Marker marker;
    marker.header.frame_id = global_costmap_->getGlobalFrameID();
    marker.header.stamp = ros::Time::now();
    marker.ns = "goal_pose";
    marker.id = 0;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.a = 1;
    marker.color.r = 0;
    marker.color.g = 1;
    marker.color.b = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.pose = goal.pose;
    marker.lifetime = ros::Duration(0.0);
    publisher.publish(marker);

}

/// the current plan is also published by the global planner
void MoveBase::publishCurrentPlan(const std::vector<geometry_msgs::PoseStamped>& path, double r, double g, double b, double a){
    /// create a message for the plan
    visualization_msgs::Marker current_path;
    current_path.header.frame_id = global_costmap_->getGlobalFrameID();
    current_path.header.stamp = ros::Time::now();
    current_path.ns = "current_plan";
    current_path.type = visualization_msgs::Marker::LINE_STRIP;
    current_path.scale.x = global_costmap_->getResolution()/2;
    current_path.color.r = r;
    current_path.color.g = g;
    current_path.color.b = b;
    current_path.color.a = a;
    current_path.action = visualization_msgs::Marker::ADD;
    current_path.pose.orientation.w = 1.0;
    current_path.lifetime = ros::Duration(0.0);


    /// extract the plan in world co-ordinates, we assume the path is all in the same frame
    for(unsigned int i=0; i < path.size(); i++){
        geometry_msgs::Point p;
        p.x = path[i].pose.position.x;
        p.y = path[i].pose.position.y;
        p.z = 0.05;
        current_path.points.push_back(p);
    }
    current_plan_pub_.publish(current_path);
}

void MoveBase::publishRePlan(const std::vector<geometry_msgs::PoseStamped>& path, double r, double g, double b, double a){
    /// create a message for the plan
    visualization_msgs::Marker replan_path;

    replan_path.header.frame_id = global_costmap_->getGlobalFrameID();;
    replan_path.header.stamp = ros::Time::now();
    replan_path.ns = "re_plan";
    replan_path.type = visualization_msgs::Marker::LINE_STRIP;
    replan_path.scale.x =  global_costmap_->getResolution()/2;
    replan_path.color.r = r;
    replan_path.color.g = g;
    replan_path.color.b = b;
    replan_path.color.a = a;
    replan_path.pose.orientation.w = 1.0;
    replan_path.lifetime = ros::Duration(0.0);
    if (!path.empty())
        replan_path.action = visualization_msgs::Marker::ADD;
    else
        replan_path.action = visualization_msgs::Marker::DELETE;

    /// extract the plan in world co-ordinates, we assume the path is all in the same frame
    for(unsigned int i=0; i < path.size(); i++){
        geometry_msgs::Point p;
        p.x = path[i].pose.position.x;
        p.y = path[i].pose.position.y;
        p.z = 0.05;
        replan_path.points.push_back(p);
    }
    re_plan_pub_.publish(replan_path);
}

void MoveBase::publishDebugMarker(geometry_msgs::Point &point, std_msgs::ColorRGBA &color, ros::Publisher& publisher) {

    visualization_msgs::Marker marker;
    marker.header.frame_id = global_costmap_->getGlobalFrameID();
    marker.header.stamp = ros::Time::now();
    marker.ns = "goal_pose";
    marker.id = 0;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color = color;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.pose.position = point;
    marker.lifetime = ros::Duration(0.1);
    publisher.publish(marker);
}

};

int main(int argc, char** argv){
    ros::init(argc, argv, "move_base_node");
    tf::TransformListener tf(ros::Duration(10));

    move_base::MoveBase move_base("move_base", tf);
    //ros::spin();

    ros::Rate r(20);
    while (ros::ok()) {
        ros::spinOnce();
        move_base.generateReference();
        r.sleep();
    }


    return(0);
}
