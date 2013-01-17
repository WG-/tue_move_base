#ifndef NAV_MOVE_BASE_ACTION_H_
#define NAV_MOVE_BASE_ACTION_H_

#include <ros/ros.h>

// messages, services and actions
#include <tue_move_base_msgs/MoveBaseAction.h>
#include <tue_move_base_msgs/GetPath.h>
#include <tue_costmap_msgs/PointQuery.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>

// plugins
#include <nav_core/base_local_planner.h>
#include <nav_core/base_global_planner.h>
#include <pluginlib/class_loader.h>

// costmap
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

//#include <tue_map_3d/Map3D.h>

// tools
#include <actionlib/server/simple_action_server.h>
#include <tf/transform_listener.h>

// std
#include <vector>
#include <string>
#include <std_msgs/ColorRGBA.h>

using namespace std;

namespace move_base {

typedef actionlib::SimpleActionServer<tue_move_base_msgs::MoveBaseAction> MoveBaseActionServer;

class MoveBase {
public:

    MoveBase(std::string name, tf::TransformListener& tf);

    virtual ~MoveBase();

    void generateReference();

private:

    bool queryCostmapPointService(tue_costmap_msgs::PointQuery::Request& req, tue_costmap_msgs::PointQuery::Response& resp);

    bool planService(tue_move_base_msgs::GetPath::Request &req, tue_move_base_msgs::GetPath::Response &resp);

    bool makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

    void simpleCallback(const geometry_msgs::PoseStamped::ConstPtr& goal);

    void goalCallback();

    void preemtCallback();

    int determineCurrentWaypoint();

    bool checkGlobalPath(geometry_msgs::PointStamped& obstacle_position, bool &goal_blocked);

    bool updateGoal(geometry_msgs::PoseStamped& goal);

    void publishCostmap(const costmap_2d::Costmap2DROS& costmap_ros, int size, ros::Publisher& publisher);

    void publishGoal(geometry_msgs::PoseStamped &goal, ros::Publisher& publisher);

    void publishDebugMarker(geometry_msgs::Point &point, std_msgs::ColorRGBA &color, ros::Publisher& publisher);

    tf::TransformListener& tf_;

    tf::Stamped<tf::Pose> robot_pose_;

    MoveBaseActionServer* as_;

    float goal_area_radius_;

    std::vector<geometry_msgs::PoseStamped> current_path_, replanned_path_;
    geometry_msgs::PoseStamped current_goal_;
    int current_waypoint_;

    //nav_core::BaseLocalPlanner local_planner_;    // deprecated, the plugin
    //nav_core::BaseLocalPlanner global_planner_;

    boost::shared_ptr<nav_core::BaseLocalPlanner> local_planner_;
    boost::shared_ptr<nav_core::BaseGlobalPlanner> global_planner_;

    std::string robot_base_frame_, global_frame_;

    tf::Stamped<tf::Pose> global_pose_;

    ros::Time t_last_cmd_vel;
    ros::Duration time_to_replan_;

    ros::Publisher current_goal_pub_, vel_pub_, action_goal_pub_, debug_marker_pub_, goal_pose_pub_;
    ros::Publisher pub_local_costmap_, pub_global_costmap_, pub_new_costmap_;
    ros::Subscriber goal_sub_;
    ros::ServiceServer make_plan_srv_;
    ros::ServiceServer query_costmap_srv_;

    pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;
    pluginlib::ClassLoader<nav_core::BaseLocalPlanner> blp_loader_;

    costmap_2d::Costmap2DROS* global_costmap_;
    // from now on we will only use the global costmap
    //costmap_2d::Costmap2DROS* local_costmap_;
};
};
#endif

