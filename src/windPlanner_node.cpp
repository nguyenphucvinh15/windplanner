#include <ros/ros.h>
#include "windplanner/rhe.hpp"
#include "windplanner/rrt.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "rhep");
    ros::NodeHandle nh;

    windPlanner::Planner planner(nh);

    ros::spin();
    return 0;
}
