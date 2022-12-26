#ifndef COLLISION_DETECTOR_HPP_  // NOLINT
#define COLLISION_DETECTOR_HPP_

#include <ros/ros.h>
#include <Eigen/Dense>
#include <nav_msgs/OccupancyGrid.h>

#include <vector>

#include "windplanner/node.hpp"


namespace windPlanner {
    
    class CollisionDetector {
        public:
            enum CellProperty { ERROR = 0, OPEN = -1, COLLIDE = 1, UNKNOWN = 2, FREE = 3, FREE_OPEN = 4, COLLIDE_OPEN = 5, FREE_COLLIDE_OPEN = 6 };
            enum CellStatus { kError = 0, kFree = 1, kOccupied = 2, kUnknown = -1 };

            CollisionDetector() = default;

            CellProperty thisPointProperties(double wx, double wy);
            CellProperty thisPointProperties(double wx, double wy, int8_t& cost);
            CellProperty thisPointProperties(const Eigen::Vector2d& point);
            CellProperty thisPointProperties(const Eigen::Vector2d& point, int8_t& cost);

            CellStatus thisPointStatus(double wx, double wy);
            CellStatus thisPointStatus(double wx, double wy, int8_t& cost);
            CellStatus thisPointStatus(const Eigen::Vector2d& point);
            CellStatus thisPointStatus(const Eigen::Vector2d& point, int8_t& cost);
            CellStatus thisPointStatusBoundingBox(const Eigen::Vector2d& point, const Eigen::Vector2d& bounding_box_size);
            CellStatus thisPointStatusBoundingBox(const Eigen::Vector2d& point, const Eigen::Vector2d& bounding_box_size, int8_t& cost);

            // line between point and point
            CellStatus whatBetween(const Eigen::Vector2d& start, const Eigen::Vector2d& end);
            CellStatus whatBetweenBoundingBox(const Eigen::Vector2d& start, const Eigen::Vector2d& end, const Eigen::Vector2d& bounding_box_size);

            void setMap(nav_msgs::OccupancyGrid& costmap);

        private:
            void worldToMap(double wx, double wy, uint32_t& mx, uint32_t& my);  // NOLINT
            void mapToWorld(uint32_t mx, uint32_t my, double& wx, double& wy);
            int8_t getGridValue(uint32_t mx, uint32_t my);

            nav_msgs::OccupancyGrid costmap_;
            double resolution_{0.1};
            uint32_t size_x_{0};
            uint32_t size_y_{0};
            double origin_x_{0.0};
            double origin_y_{0.0};
            std::vector<int8_t> data_;
    };

}  // namespace windPlanner

#endif  // COLLISION_DETECTOR_HPP_  NOLINT
