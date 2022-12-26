#ifndef RHE_PLANNER_HPP_  // NOLINT
#define RHE_PLANNER_HPP_

#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

#include "windplanner/rrt.hpp"
#include "windplanner/pe.hpp"
#include "windplanner/rhep_srv.h"
#include "windplanner/rhep_resrv.h"


namespace windPlanner {
    class Planner {
        private:
            ros::NodeHandle nh_;
            ros::ServiceServer plannerService_;
            ros::ServiceServer rePlannerService_;
            ros::Subscriber posClient_;
            ros::Subscriber odomClient_;
            ros::Subscriber mapClient_;
            ros::Publisher weightMapPub_;
            ros::Publisher robPosPub_;
            ros::Publisher pathPub_;

            tf::TransformListener listener_;
            tf::StampedTransform transform_;

            nav_msgs::OccupancyGrid map_;

            RRT* tree_;
            PE* rePlanner_;
            CollisionDetector cd_;

            RRT::Params params_;
            PE::Params paramsRePlanner_;
            
            double resolution_;

            bool initialized_{false};
            bool mapReady_{false};
            

        public:
            Planner(const ros::NodeHandle& nh);

            bool setParams();

            void posCallback(const geometry_msgs::PoseWithCovarianceStamped& pose);
            void odomCallback(const nav_msgs::Odometry& pose);
            void mapCallback(const nav_msgs::OccupancyGrid& msg);

            bool plannerCallback(windplanner::rhep_srv::Request& req, windplanner::rhep_srv::Response& res);
            bool rePlannerCallback(windplanner::rhep_resrv::Request& req, windplanner::rhep_resrv::Response& res);

    };

}  // namespace windPlanner

#endif  // RHE_PLANNER_HPP_  // NOLINT
