#include <move_base/move_base.h>
#include <visualization_msgs/Marker.h>
#include <boost/algorithm/string.hpp>

namespace move_base {

MoveBase::MoveBase(std::string name, tf::TransformListener& tf) :
    tf_(tf),
    as_(NULL),
    local_planner_(NULL),
    global_planner_(NULL),
    bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"),
    blp_loader_("nav_core", "nav_core::BaseLocalPlanner") {

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

    pub_local_costmap_ = private_nh.advertise<visualization_msgs::Marker>("local_costmap", 2);
    pub_global_costmap_ = private_nh.advertise<visualization_msgs::Marker>("global_costmap", 2);


    //for commanding the base
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    current_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_goal", 0 );

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

    local_costmap_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
    local_costmap_->pause();

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
        global_planner_ = bgp_loader_.createClassInstance(global_planner);
        global_planner_->initialize(bgp_loader_.getName(global_planner), global_costmap_);
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
        local_planner_ = blp_loader_.createClassInstance(local_planner);
        local_planner_->initialize(blp_loader_.getName(local_planner), &tf_, local_costmap_);
    } catch (const pluginlib::PluginlibException& ex)
    {
        ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", local_planner.c_str(), ex.what());
        exit(0);
    }

    // Start actively updating costmaps based on sensor data
    local_costmap_->start();
    global_costmap_->start();

    //advertise a service for getting a plan
    make_plan_srv_ = private_nh.advertiseService("get_plan", &MoveBase::planService, this);

    query_costmap_srv_ = private_nh.advertiseService("query_costmap", &MoveBase::queryCostmapPointService, this);

    //we're all set up now so we can start the action server
    as_->start();
}

MoveBase::~MoveBase(){

    if (as_) {
        delete as_;
    }

    if (global_planner_) {
        delete global_planner_;
    }

    if (local_planner_) {
        delete local_planner_;
    }

    delete global_costmap_;
    delete local_costmap_;
}

/**********************************************************
 *                      CALLBACKS
 **********************************************************/

void MoveBase::simpleCallback(const geometry_msgs::PoseStamped::ConstPtr& goal){
    ROS_DEBUG_NAMED("move_base","In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");

    geometry_msgs::PoseStamped robot_pose_msg;
    robot_pose_msg.header.frame_id = global_frame_;
    robot_pose_msg.header.stamp = ros::Time();

    global_costmap_->getRobotPose(robot_pose_);
    tf::poseStampedTFToMsg(robot_pose_, robot_pose_msg);

    tue_move_base_msgs::MoveBaseActionGoal action_goal;
    action_goal.header.stamp = ros::Time::now();

    if (global_planner_->makePlan(robot_pose_msg, *goal, action_goal.goal.path)) {
        action_goal_pub_.publish(action_goal);
    }
}

void MoveBase::goalCallback() {
    tue_move_base_msgs::MoveBaseGoal goal = *as_->acceptNewGoal();
    current_path_ = goal.path;
    local_planner_->setPlan(current_path_);
    current_goal_ = goal.path.back();
    ros::Time t_last_cmd_vel = ros::Time::now();
}

void MoveBase::preemtCallback() {
    as_->setPreempted();
}

bool MoveBase::queryCostmapPointService(tue_costmap_msgs::PointQuery::Request& req, tue_costmap_msgs::PointQuery::Response& resp) {

    costmap_2d::Costmap2D costmap;
    global_costmap_->getCostmapCopy(costmap);

    for(vector<geometry_msgs::PointStamped>::const_iterator it_point = req.points.begin(); it_point != req.points.end(); ++it_point) {
        tf::Stamped<tf::Point> point;
        tf::pointStampedMsgToTF(*it_point, point);
        point.stamp_ = ros::Time();

        tf::Stamped<tf::Point> point_GLOBAL;
        tf_.transformPoint(global_costmap_->getGlobalFrameID(), point, point_GLOBAL);

        unsigned int mx, my;
        costmap.worldToMap(point_GLOBAL.getX(), point_GLOBAL.getY(), mx, my);

        unsigned char cost = costmap.getCost(mx, my);

        tue_costmap_msgs::PointInfo point_info;
        point_info.point = *it_point;

        // todo: better distance representation
        point_info.distance_to_closest_obstacle = 1 - (double)cost / 255;

        resp.points_info.push_back(point_info);
    }

    return true;
}

bool MoveBase::planService(tue_move_base_msgs::GetPath::Request &req, tue_move_base_msgs::GetPath::Response &res) {

    ROS_INFO("planService");

    geometry_msgs::PoseStamped robot_pose_msg;
    robot_pose_msg.header.frame_id = global_frame_;
    robot_pose_msg.header.stamp = ros::Time();

    global_costmap_->getRobotPose(robot_pose_);
    tf::poseStampedTFToMsg(robot_pose_, robot_pose_msg);

    req.target_pose.header.stamp = ros::Time();

    global_planner_->makePlan(robot_pose_msg, req.target_pose, res.path);

    return true;
}

/**********************************************************
 *
 **********************************************************/


void MoveBase::generateReference() {

    // clear the local and the global costmap
    // so only the static map + the latest sensor readings are in the global costmap
    // and only the latest sensor readings are in the local costmap
    local_costmap_->resetMapOutsideWindow(0,0);
    global_costmap_->resetMapOutsideWindow(0,0);

    publishCostmap(*local_costmap_, 100, pub_local_costmap_);
    publishCostmap(*global_costmap_, 100, pub_global_costmap_);

    if (!as_->isActive()) {
        return;
    }

    if (local_planner_->isGoalReached()) {
        as_->setSucceeded();
    }

    // ask the local planner to compute a velocity
    geometry_msgs::Twist cmd_vel;
    if (local_planner_->computeVelocityCommands(cmd_vel)) {
        vel_pub_.publish(cmd_vel);
        t_last_cmd_vel = ros::Time::now();
    }

    // else if no velocity can be computed a re-plan is requested
    else if (!local_planner_->computeVelocityCommands(cmd_vel))
    {
        // if the search for a re-plan takes too long the action client is aborted
        ROS_INFO_STREAM("Time Now = " << ros::Time::now() << "  Last Cmd Vel = " << t_last_cmd_vel);
        if ((ros::Time::now() - t_last_cmd_vel) > ros::Duration(10.0)) {
            as_->setAborted();
        }
        // else, try to re-plan
        else {
            // if a velocity command can not be computed, do a re-plan
            geometry_msgs::PoseStamped robot_pose_msg;
            robot_pose_msg.header.frame_id = global_frame_;
            robot_pose_msg.header.stamp = ros::Time();
            global_costmap_->getRobotPose(robot_pose_);
            tf::poseStampedTFToMsg(robot_pose_, robot_pose_msg);
            current_path_.clear();
            // if a new plan can be found, send it to the local planner
            if (global_planner_->makePlan(robot_pose_msg, current_goal_, current_path_) && !local_planner_->isGoalReached()) {
                ROS_INFO("I found a re-plan and I will hand it off to the local planner");
                local_planner_->setPlan(current_path_);
            }
        }
    }

    // generate a feedback message with the current base position
    // and the percentage of the path that is completed
    global_costmap_->getRobotPose(robot_pose_);
    tue_move_base_msgs::MoveBaseFeedback feedback_msg;
    // directly transform the robot pose to the feedback message
    tf::poseStampedTFToMsg(robot_pose_, feedback_msg.base_position);

    // determine the percentage of the path that is covered so far
    // as long as the goal is not reached
    double ratio;
    if (!local_planner_->isGoalReached()){
        ratio = (double)determineCurrentWaypoint()/current_path_.size();
        feedback_msg.percent_complete = ratio;
    }
    // if the goal is reached, set the percentage complete to 1
    else if (local_planner_->isGoalReached()) {
        ratio = 1;
        feedback_msg.percent_complete = ratio;
    }

    as_->publishFeedback(feedback_msg);
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
    marker.ns = "costmap_amigo_global";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = costmap.getResolution();
    marker.scale.y = costmap.getResolution();
    marker.scale.z = costmap.getResolution();

    // Set alpha to 1
    marker.color.a = 1;

    marker.lifetime = ros::Duration(0.0);

    double x, y;
    for(int mx = max((int)mx_center - size, 0); mx < min((int)mx_center + size, width - 1); ++mx) {
        for(int my = max((int)my_center - size, 0); my < min((int)my_center + size, height - 1); ++my) {
            double cost = (double)costmap.getCost((int)mx, (int)my) / 255;

            costmap.mapToWorld(mx, my, x, y);

            geometry_msgs::Point p;
            p.x = x;
            p.y = y;
            p.z = -0.05;

            std_msgs::ColorRGBA color;
            color.r = cost;
            color.g = 0;
            color.b = 1 - cost;
            color.a = 0.95;

            marker.points.push_back(p);
            marker.colors.push_back(color);
        }
    }

    publisher.publish(marker);
}

};

int main(int argc, char** argv){
    ros::init(argc, argv, "move_base_node");
    tf::TransformListener tf(ros::Duration(10));

    move_base::MoveBase move_base("move_base", tf);
    //ros::spin();

    ros::Rate r(10);
    while (ros::ok()) {
        ros::spinOnce();
        move_base.generateReference();
        r.sleep();
    }


    return(0);
}
