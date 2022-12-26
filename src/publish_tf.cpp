#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>


nav_msgs::Odometry odomMSG;
geometry_msgs::PoseStamped poseMSG;
tf::Quaternion quatTF(0.0, 0.0, 0.0, 1.0);
tf::Vector3 posTF(0.0, 0.0, 0.0);

void odomCallback(const nav_msgs::Odometry& msg) { 
    odomMSG = msg;
    tf::quaternionMsgToTF(odomMSG.pose.pose.orientation, quatTF);
    posTF = tf::Vector3(odomMSG.pose.pose.position.x, odomMSG.pose.pose.position.y, 0.0);
}

void poseCallback(const geometry_msgs::PoseStamped& msg) {
    poseMSG = msg;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Controller");

    ros::NodeHandle nh;
    ros::Subscriber odomSub = nh.subscribe("/odom", 1000, &odomCallback);
    ros::Subscriber poseSub = nh.subscribe("/tracked_pose", 1000, &poseCallback);

    ros::Rate rate(50);
    float dElapse = ros::Duration(rate).toSec();

    while (odomMSG.header.seq == 0) {
        rate.sleep();
        ros::spinOnce();
    }

    tf::TransformBroadcaster br;
    static tf::TransformListener listener;

    while (ros::ok()) {
        ros::Time nowTime = ros::Time::now();

        tf::Transform transform(quatTF, posTF);
        br.sendTransform(tf::StampedTransform(transform, nowTime, "odom", "base_link"));

        if (poseMSG.header.seq > 0) {
            tf::StampedTransform transformS;
            
            listener.waitForTransform("imu_link", "odom", ros::Time(0), ros::Duration(10.0));
            listener.lookupTransform("imu_link", "odom", ros::Time(0), transformS);
            
            tf::Pose poseTF;
            tf::poseMsgToTF(poseMSG.pose, poseTF);
            tf::Vector3 posPoseTF = poseTF.getOrigin();
            posPoseTF = transformS * posPoseTF;
            tf::Quaternion quatPoseTF = poseTF.getRotation();
            quatPoseTF = transformS * quatPoseTF;

            tf::Transform transform(quatPoseTF, posPoseTF);
            br.sendTransform(tf::StampedTransform(transform, nowTime, "map", "odom"));
        }

        rate.sleep();
        ros::spinOnce();
    }
}
