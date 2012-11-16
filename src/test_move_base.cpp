#include <ros/ros.h>

// Action client
#include <actionlib/client/simple_action_client.h>

// messages, services and actions
#include <tue_move_base_msgs/MoveBaseAction.h>
#include <tue_move_base_msgs/GetPath.h>

#include <tue_costmap_msgs/PointQuery.h>

using namespace std;

int main(int argc, char** argv){
    ros::init(argc, argv, "test_move_base");

    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<tue_move_base_msgs::GetPath>("/move_base/get_plan");

    tue_move_base_msgs::GetPath srv;
    srv.request.target_pose.header.frame_id = "/map";
    srv.request.target_pose.pose.position.x = 0;
    srv.request.target_pose.pose.position.z = 0;
    srv.request.target_pose.pose.position.y = 0;
    srv.request.target_pose.pose.orientation.x = 0;
    srv.request.target_pose.pose.orientation.y = 0;
    srv.request.target_pose.pose.orientation.z = 0;
    srv.request.target_pose.pose.orientation.w = 1;

    if (client.call(srv)) {
        if (srv.response.path.empty()) {
            ROS_ERROR("No path found.");
        } else {
            for(vector<geometry_msgs::PoseStamped>::iterator it = srv.response.path.begin(); it != srv.response.path.end(); ++it) {
                cout << *it << endl;
            }
        }
    } else {
        ROS_ERROR("Path request failed");
        return -1;
    }

    ros::ServiceClient client_costmap = nh.serviceClient<tue_costmap_msgs::PointQuery>("/move_base/query_costmap");

    tue_costmap_msgs::PointQuery srv_costmap;    

    geometry_msgs::PointStamped pnt1;
    pnt1.header.frame_id = "/map";
    pnt1.point.x = 3;
    pnt1.point.y = 2;
    pnt1.point.z = 0;
    srv_costmap.request.points.push_back(pnt1);

    if (client_costmap.call(srv_costmap)) {
        if (srv_costmap.response.points_info.empty()) {
            ROS_INFO("Empty point info received");
        } else {
            for(vector<tue_costmap_msgs::PointInfo>::iterator it_pnt = srv_costmap.response.points_info.begin(); it_pnt != srv_costmap.response.points_info.end(); ++it_pnt) {
                cout << *it_pnt << endl;
            }
        }
    } else {
        ROS_ERROR("Calling service %s failed", client_costmap.getService().c_str());
        return -1;
    }


    /*
    actionlib::SimpleActionClient<tue_move_base_msgs::MoveBaseAction> move_base("move_base", true);
    move_base.waitForServer();

    // move base
    tue_move_base_msgs::MoveBaseGoal base_goal;
    base_goal.path = srv.response.path;

    move_base.sendGoal(base_goal);
    move_base.waitForResult(ros::Duration(50.0));
    */

    return 0;

}
