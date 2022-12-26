#ifndef PE_HPP_  // NOLINT
#define PE_HPP_

#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <vector>

#include "windplanner/random_double_generator.hpp"
#include "windplanner/collision_detector.hpp"
#include "windplanner/node.hpp"
#include "windplanner/rrt.hpp"


namespace windPlanner {
    class PE {

        public:
            struct Params {
                uint32_t numberSamples;
                Eigen::Vector2d boundingBox;
                double replanRange;
                double sampleRange;

                rviz_visual_tools::RvizVisualToolsPtr visualizeEstimates_;
                rviz_visual_tools::RvizVisualToolsPtr visualizeLandMarks_;
                rviz_visual_tools::RvizVisualToolsPtr visualizeSamples_;
                rviz_visual_tools::RvizVisualToolsPtr visualizeReWeightedSamples_;
            };

            std::vector<geometry_msgs::Pose> initialize();
            std::vector<geometry_msgs::Pose> iterate();
            Eigen::Vector2d getLandMarks();
            std::vector<double> predictEstimate(std::vector<double> weight);
            geometry_msgs::Pose reSampling(std::vector<double> weight);
            Eigen::Vector2d calculateEstimate(std::vector<Eigen::Vector2d> matA, std::vector<double> matB);
            
            void publishEstimates(Eigen::Vector2d state);
            void publishLandMarks(Eigen::Vector2d state);
            void publishSamples(Eigen::Vector2d state);
            void publishReWeightedSamples(Eigen::Vector2d state);
            void publishEstTrees();

            void setParams(Params &params);
            void setPaths(std::vector<geometry_msgs::Pose> reqPath);
            void setMap(const nav_msgs::OccupancyGrid& msg);

        private:
            std::size_t goalLoc_;

            Eigen::Vector2d xMeas_;
            std::vector<Eigen::Vector2d> xOrigins_;
            std::vector<Eigen::Vector2d> x_est_;
            std::vector<Eigen::Vector2d> landMarks_;
            std::vector<std::vector<Eigen::Vector2d>> vec_px_;
            std::vector<std::vector<double>> weight_;

            std::vector<geometry_msgs::Pose> reqPath_;
            CollisionDetector cd_;
            nav_msgs::OccupancyGrid map_;

            double R_;
            double rA_;
        
        protected:
            Params params_;
    };
}

#endif  // PE_HPP_  NOLINT