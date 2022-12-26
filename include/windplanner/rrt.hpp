#ifndef RRT_HPP_  // NOLINT
#define RRT_HPP_

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <vector>
#include <stack>

#include "windplanner/node.hpp"
#include "windplanner/random_double_generator.hpp"
#include "windplanner/collision_detector.hpp"
#include "windplanner/TreeMsg.h"


namespace windPlanner {

    class RRT {

        public:
            struct Params {
                double v_max_;
                double dyaw_max_;
                double dOvershoot_;
                
                double dt_;
                double igProbabilistic_;
                double igFree_;
                double igOccupied_;
                double igUnmapped_;
                double gainRange_;
                double degressiveCoeff_;
                double zero_gain_;
                
                double extensionRange_;
                double minExtensionRange_;
                bool exact_root_;
                int32_t initIterations_;
                int32_t cuttoffIterations_;
                int32_t numberSamples_;

                Eigen::Vector2d boundingBox_;
                Eigen::Vector2d mapWeightBox_;

                std::string mapFrame_;
                std::string baseFrame_;
                std::string poseTopic_;
                std::string odomTopic_;
                std::string mapTopic_;

                rviz_visual_tools::RvizVisualToolsPtr visualizeTrees_;
                rviz_visual_tools::RvizVisualToolsPtr visualizeTreePoints_;
            };

            RRT();
            ~RRT();

            virtual void setStateFromPoseMsg(const geometry_msgs::PoseWithCovarianceStamped& pose);
            virtual void setStateFromOdometryMsg(const nav_msgs::Odometry& pose);
            virtual geometry_msgs::PoseWithCovarianceStamped setStateFromMapMsg(const nav_msgs::OccupancyGrid& msg);
            virtual void initialize();
            virtual void iterate();
            virtual void clear();
            virtual void memorizeBestBranch();
            virtual std::vector<geometry_msgs::Pose> getBestEdge(std::string targetFrame);
            virtual std::vector<geometry_msgs::Pose> getPathBackToPrevious(std::string targetFrame);

            Eigen::Vector3d sampleFree();
            Node* getNearestNode(Eigen::Vector3d state);
            double computeGain(Eigen::Vector3d state);
            
            geometry_msgs::Pose sampleGoal(Eigen::Vector3d state, std::string targetFrame);

            void slideMapWeight();
            std::vector<double> calculateWeight();
            void visualizeWeight();

            void publishNode(Node* node);
            void publishTreePoints(Eigen::Vector3d state);

            std::vector<Node*> getNodes() const;
            nav_msgs::OccupancyGrid getWeightMap();

            void setParams(Params &params);
            uint32_t getCounter();
            bool gainFound();

        private:
            windplanner::TreeMsg* kdTree_;
            std::stack<Eigen::Vector3d> history_;
            std::vector<Eigen::Vector3d> bestBranchMemory_;
            uint64_t iterationCount_;

            nav_msgs::OccupancyGrid costmap_;
            nav_msgs::OccupancyGrid weightMap_;
            std::vector<Node*> nodes_;
            RandomDoubleGenerator random_double_;
            std::vector<double> mapWeight_;
            std::vector<double> weight_;

            CollisionDetector cd_;

            double init_x_;
            double init_y_;
            double resolution_;
            bool isNeedWeightSampling_{false};
        
        protected:
            Params params_;
            uint32_t counter_;
            double bestGain_;
            Node* bestNode_;
            Node* rootNode_;
            Eigen::Vector3d root_;
            Eigen::Vector3d exact_root_;
    };

}  // namespace windPlanner

#endif  // RRT_HPP_  NOLINT
