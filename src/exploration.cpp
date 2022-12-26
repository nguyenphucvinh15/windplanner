#include <thread>
#include <chrono>

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "windplanner/rhep_srv.h"
#include "windplanner/rhep_resrv.h"


// Global variables
nav_msgs::OccupancyGrid mapData;
visualization_msgs::Marker points, line;


int main(int argc, char** argv) {
    ros::init(argc, argv, "exploration");
    ros::NodeHandle nh;

    ros::Rate rate(30);

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action("/move_base", true);
    action.waitForServer();

    move_base_msgs::MoveBaseGoal goal;
    windplanner::rhep_srv planSrv;
    windplanner::rhep_resrv planReSrv;
    
    goal.target_pose.header.frame_id = "map";
    planSrv.request.header.frame_id = "map";
    bool stateFlag = true;

    // Start planning: The planner is called and the computed path sent to the controller.
    uint32_t iteration = 0;
    ros::Time startTime = ros::Time::now();

    while (ros::ok()) {
        if (stateFlag) {
            ROS_INFO("Start Planner!");
            if (ros::service::call("rheplanner", planSrv) && planSrv.response.path.size() != 0) {
                planReSrv.request.path = planSrv.response.path;
                planSrv.response.path.clear();
                planReSrv.request.init.data = true;

                goal.target_pose.header.stamp = ros::Time::now();
                goal.target_pose.pose = planReSrv.request.path.back();
                planReSrv.request.path.pop_back();
                action.sendGoal(goal);

                if (ros::service::call("rhereplanner", planReSrv)) {
                    planReSrv.request.init.data = false;
                    stateFlag = false;

                    ROS_INFO("Planning iteration %i", iteration);
                    iteration++;
                }
            }
            else {
                ROS_WARN("Planner not reachable");
                continue;
            }
        }
        
        if (planReSrv.response.path.size() != 0 && !stateFlag) {
            if (action.getState() == actionlib::SimpleClientGoalState::ACTIVE) {
                planReSrv.request.path = planReSrv.response.path;
                if (ros::service::call("rhereplanner", planReSrv)) {
                    ROS_DEBUG("Start Replanning State!");
                }
                else {
                    ROS_ERROR("Cannot Replanning!");
                }
            }
            if (action.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                goal.target_pose.header.stamp = ros::Time::now();
                goal.target_pose.pose = planReSrv.response.path.back();
                planReSrv.response.path.pop_back();
                action.sendGoal(goal);
            }
            action.waitForResult(ros::Duration(rate));
        }
        else if (action.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            stateFlag = true;
        }
        
        // if (ros::Time::now() - startTime > ros::Duration(180.0)) {
        //     ros::shutdown();
        // }

        ros::spinOnce();
		rate.sleep();
    }
}
