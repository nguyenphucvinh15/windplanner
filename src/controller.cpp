#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32.h>
#include <tf/tf.h>


nav_msgs::Odometry odomMSG;
sensor_msgs::Imu imuMSG;
tf::Quaternion quatIMU;
float leftEncoderRPM, rightEncoderRPM;

const float WHEEL_BASE = 0.288; // (m) Khoang cach 2 banh xe
const float RPM2RPS = M_PI / 60.0; // Vong/phut --> radian/s 
const float R_WHEEL = 0.0792; // (m) Ban kinh banh xe

ros::Publisher rpmLeftPub, rpmRightPub, odomPub, imuPub;

void velocityCallback(const geometry_msgs::Twist& msg) {
    // Using the callback function just for subscribing  
    // Subscribing the message and storing it in 'linx' and 'angZ'
    std_msgs::Int32 rpmLeft;
    std_msgs::Int32 rpmRight;

    rpmLeft.data = static_cast<int32_t>(std::round((msg.linear.x - 0.5 * msg.angular.z * WHEEL_BASE) / (RPM2RPS * R_WHEEL) * 21.5));
    rpmRight.data = static_cast<int32_t>(-std::round((msg.linear.x + 0.5 * msg.angular.z * WHEEL_BASE) / (RPM2RPS * R_WHEEL) * 21.5));

    rpmLeftPub.publish(rpmLeft);
    rpmRightPub.publish(rpmRight);
}

void leftEncoderCallback(const std_msgs::Int32& msg) {
    // Using the callback function just for subscribing  
    // Subscribing the message and storing it in 'linx' and 'angZ'
    leftEncoderRPM = static_cast<float>(msg.data / 21.5);
}

void rightEncoderCallback(const std_msgs::Int32& msg) {
    // Using the callback function just for subscribing  
    // Subscribing the message and storing it in 'linx' and 'angZ'
    rightEncoderRPM = static_cast<float>(-msg.data / 21.5);
}

void imuCallback(const sensor_msgs::Imu& msg) {
    imuMSG = msg;
    imuMSG.header.frame_id = "imu_link";
    imuMSG.angular_velocity.x = 0.0;
    imuMSG.angular_velocity.y = 0.0;
    imuMSG.angular_velocity.z = msg.angular_velocity.y;
    imuMSG.linear_acceleration.x = msg.linear_acceleration.z;
    imuMSG.linear_acceleration.y = msg.linear_acceleration.x;
    imuMSG.linear_acceleration.z = msg.linear_acceleration.y;

    tf::quaternionMsgToTF(imuMSG.orientation, quatIMU);

    double roll, pitch, yaw;
    tf::Matrix3x3(quatIMU).getRPY(roll, pitch, yaw);

    quatIMU.setRPY(0.0, 0.0, pitch);
    tf::quaternionTFToMsg(quatIMU, imuMSG.orientation);

    imuPub.publish(imuMSG);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Controller");

    ros::NodeHandle nh;
    ros::Subscriber velSub = nh.subscribe("/cmd_vel", 1000, &velocityCallback);
    ros::Subscriber lEncoderSub = nh.subscribe("/leftEncoder", 1000, &leftEncoderCallback);
    ros::Subscriber rEncoderSub = nh.subscribe("/rightEncoder", 1000, &rightEncoderCallback);
    ros::Subscriber imuSub = nh.subscribe("/imu/data", 1000, &imuCallback);
    rpmLeftPub = nh.advertise<std_msgs::Int32>("/rpmLeft", 1000);
    rpmRightPub = nh.advertise<std_msgs::Int32>("/rpmRight", 1000);
    odomPub = nh.advertise<nav_msgs::Odometry>("/odom", 1000);
    imuPub = nh.advertise<sensor_msgs::Imu>("/imu/filtered", 1000);

    ros::Rate rate(100);
    float dElapse = ros::Duration(rate).toSec();

    tf::Quaternion quatTF(0.0, 0.0, 0.0, 1.0);
    tf::Vector3 posTF(0.0, 0.0, 0.0);

    static tf::TransformBroadcaster br;

    odomMSG.header.frame_id = "odom";
    odomMSG.child_frame_id = "base_footprint";

    while (ros::ok()) {
        odomMSG.header.stamp = ros::Time::now();

        float theta = tf::getYaw(quatTF);
        
        float leftEncoderDist = RPM2RPS * leftEncoderRPM * R_WHEEL * dElapse;
        float rightEncoderDist = RPM2RPS * rightEncoderRPM * R_WHEEL * dElapse;
        
        float dPos = (leftEncoderDist + rightEncoderDist) / 2;
        float dAng = (rightEncoderDist - leftEncoderDist) / WHEEL_BASE;

        double dx = dPos * cos (theta + dAng / 2.0);
        double dy = dPos * sin (theta + dAng / 2.0);
        double dtheta = dAng;
        
        odomMSG.twist.twist.linear.x = sqrt(dx*dx + dy*dy) / dElapse;
        odomMSG.twist.twist.angular.z = dtheta / dElapse;
        
        odomMSG.pose.pose.position.x += dx;
        odomMSG.pose.pose.position.y += dy;
        theta += dtheta;

        quatTF.setRPY(0.0, 0.0, theta);
        tf::quaternionTFToMsg(quatTF, odomMSG.pose.pose.orientation);
        posTF = tf::Vector3(odomMSG.pose.pose.position.x, odomMSG.pose.pose.position.y, 0.0);

        // tf::Transform transform(quatTF, posTF);
        // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));

        odomMSG.pose.covariance[0] = 0.00001;
        odomMSG.pose.covariance[7] = 0.00001;
        odomMSG.pose.covariance[14] = 1000000000000.0;
        odomMSG.pose.covariance[21] = 1000000000000.0;
        odomMSG.pose.covariance[28] = 1000000000000.0;
        odomMSG.pose.covariance[35] = 0.001;

        // It would be better to apply the conditions within the main function and use the 
        // Callback function just for subscribing
        odomPub.publish(odomMSG);

        rate.sleep();
        ros::spinOnce();
    }
}
