#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>


nav_msgs::Path path;

ros::Publisher pathPub;

void odomCallback(const nav_msgs::Odometry& msg) {
    path.header = msg.header;
    geometry_msgs::PoseStamped pose;
    pose.header = msg.header;
    pose.pose = msg.pose.pose;
    path.poses.push_back(pose);
    pathPub.publish(path);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "visualizePath");

    ros::NodeHandle nh;
    ros::Subscriber velSub = nh.subscribe("/odom", 1000, &odomCallback);
    pathPub = nh.advertise<nav_msgs::Path>("/path", 1000);

    ros::spin();
}
