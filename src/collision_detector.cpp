#include "windplanner/collision_detector.hpp"

namespace windPlanner {

    CollisionDetector::CellProperty CollisionDetector::thisPointProperties(double wx, double wy) {
        uint32_t mx, my, mz;
        this->worldToMap(wx, wy, mx, my);

        if ((mx < 0) || (my < 0) || (mx >= size_x_) || (my >= size_y_))
            return CellProperty::ERROR;

        // getCost returns unsigned char
        int8_t cost = static_cast<int8_t>(this->getGridValue(mx, my));

        if (cost == 100)
            return CellProperty::COLLIDE;
        else if (cost == -1)
            return CellProperty::UNKNOWN;
        else if (cost == 0)
            return CellProperty::FREE;
        else if ((cost > 0) && (cost < 50))
            return CellProperty::FREE_OPEN;
        else if ((cost > 50) && (cost < 100))
            return CellProperty::COLLIDE_OPEN;
        else
            return CellProperty::FREE_COLLIDE_OPEN;

        return CellProperty::ERROR;
    }

    CollisionDetector::CellProperty CollisionDetector::thisPointProperties(double wx, double wy, int8_t& cost) {
        uint32_t mx, my, mz;
        this->worldToMap(wx, wy, mx, my);

        if ((mx < 0) || (my < 0) || (mx >= size_x_) || (my >= size_y_))
            return CellProperty::ERROR;

        // getCost returns unsigned char
        cost = static_cast<int8_t>(this->getGridValue(mx, my));

        if (cost == 100)
            return CellProperty::COLLIDE;
        else if (cost == -1)
            return CellProperty::UNKNOWN;
        else if (cost == 0)
            return CellProperty::FREE;
        else if ((cost > 0) && (cost < 50))
            return CellProperty::FREE_OPEN;
        else if ((cost > 50) && (cost < 100))
            return CellProperty::COLLIDE_OPEN;
        else
            return CellProperty::FREE_COLLIDE_OPEN;

        return CellProperty::ERROR;
    }

    CollisionDetector::CellStatus CollisionDetector::thisPointStatus(double wx, double wy) {
        if (thisPointProperties(wx, wy) == CellProperty::ERROR)
            return CellStatus::kError;
        if (thisPointProperties(wx, wy) == CellProperty::UNKNOWN)
            return CellStatus::kUnknown;
        if ((thisPointProperties(wx, wy) == CellProperty::COLLIDE)
            || (thisPointProperties(wx, wy) == CellProperty::COLLIDE_OPEN))
            return CellStatus::kOccupied;
        if ((thisPointProperties(wx, wy) == CellProperty::FREE)
            || (thisPointProperties(wx, wy) == CellProperty::FREE_OPEN))
            return CellStatus::kFree;
    }

    CollisionDetector::CellStatus CollisionDetector::thisPointStatus(double wx, double wy, int8_t& cost) {
        if (thisPointProperties(wx, wy, cost) == CellProperty::ERROR)
            return CellStatus::kError;
        if (thisPointProperties(wx, wy, cost) == CellProperty::UNKNOWN)
            return CellStatus::kUnknown;
        if ((thisPointProperties(wx, wy, cost) == CellProperty::COLLIDE)
            || (thisPointProperties(wx, wy, cost) == CellProperty::COLLIDE_OPEN))
            return CellStatus::kOccupied;
        if ((thisPointProperties(wx, wy, cost) == CellProperty::FREE)
            || (thisPointProperties(wx, wy, cost) == CellProperty::FREE_OPEN))
            return CellStatus::kFree;
    }

    CollisionDetector::CellStatus CollisionDetector::thisPointStatusBoundingBox(const Eigen::Vector2d& point, const Eigen::Vector2d& bounding_box_size) {
        CellStatus center_status = thisPointStatus(point.x(), point.y());
        if ((center_status == CellStatus::kOccupied) || (center_status == CellStatus::kError)) {
            return center_status;
        }

        Eigen::Vector2d bbx_min_eigen = point - bounding_box_size / 2;
        Eigen::Vector2d bbx_max_eigen = point + bounding_box_size / 2;
        
        if (center_status == CellStatus::kFree) {
            for (double x = bbx_min_eigen.x(); x <= bbx_max_eigen.x(); x += resolution_) {
                for (double y = bbx_min_eigen.y(); y <= bbx_max_eigen.y(); y += resolution_) {
                    if (thisPointStatus(x, y) != CellStatus::kFree) {
                        return CellStatus::kError;
                    }
                }
            }
            return CellStatus::kFree;
        }
        else {
            for (double x = bbx_min_eigen.x(); x <= bbx_max_eigen.x(); x += resolution_) {
                for (double y = bbx_min_eigen.y(); y <= bbx_max_eigen.y(); y += resolution_) {
                    CellStatus ret = thisPointStatus(x, y);
                    if ((ret == CellStatus::kOccupied) || (ret == CellStatus::kError)) {
                        return ret;
                    }
                }
            }
            return CellStatus::kUnknown;
        }
    }

    CollisionDetector::CellStatus CollisionDetector::thisPointStatusBoundingBox(const Eigen::Vector2d& point, const Eigen::Vector2d& bounding_box_size, int8_t& cost) {
        CellStatus center_status = thisPointStatus(point.x(), point.y(), cost);
        if ((center_status == CellStatus::kOccupied) || (center_status == CellStatus::kError)) {
            return center_status;
        }

        Eigen::Vector2d bbx_min_eigen = point - bounding_box_size / 2;
        Eigen::Vector2d bbx_max_eigen = point + bounding_box_size / 2;
        
        if (center_status == CellStatus::kFree) {
            for (double x = bbx_min_eigen.x(); x <= bbx_max_eigen.x(); x += resolution_) {
                for (double y = bbx_min_eigen.y(); y <= bbx_max_eigen.y(); y += resolution_) {
                    if (thisPointStatus(x, y) != CellStatus::kFree) {
                        return CellStatus::kError;
                    }
                }
            }
            return CellStatus::kFree;
        }
        else {
            for (double x = bbx_min_eigen.x(); x <= bbx_max_eigen.x(); x += resolution_) {
                for (double y = bbx_min_eigen.y(); y <= bbx_max_eigen.y(); y += resolution_) {
                    CellStatus ret = thisPointStatus(x, y);
                    if ((ret == CellStatus::kOccupied) || (ret == CellStatus::kError)) {
                        return ret;
                    }
                }
            }
            return CellStatus::kUnknown;
        }
    }

    CollisionDetector::CellStatus CollisionDetector::whatBetween(const Eigen::Vector2d& start, const Eigen::Vector2d& end) {
        double dist = euclideanDistance2D(start.x(), start.y(), end.x(), end.y());
        if (dist < resolution_) {
            if (thisPointProperties(end) == CellProperty::ERROR)
                return CellStatus::kError;
            if (thisPointProperties(end) == CellProperty::UNKNOWN)
                return CellStatus::kUnknown;
            if ((thisPointProperties(end) == CellProperty::COLLIDE)
                || (thisPointProperties(end) == CellProperty::COLLIDE_OPEN))
                return CellStatus::kOccupied;
            if ((thisPointProperties(end) == CellProperty::FREE)
                || (thisPointProperties(end) == CellProperty::FREE_OPEN))
                return CellStatus::kFree;
        }
        else {
            std::size_t steps_number = static_cast<std::size_t>(floor(dist / resolution_));
            double theta = atan2(end.y() - start.y(), end.x() - start.x());
            Eigen::Vector2d p_n;
            for (std::size_t n = 1; n < steps_number; n++) {
                p_n.x() = start.x() + n*resolution_*cos(theta);
                p_n.y() = start.y() + n*resolution_*sin(theta);
                if (thisPointProperties(p_n) == CellProperty::ERROR)
                    return CellStatus::kError;
                if (thisPointProperties(p_n) == CellProperty::UNKNOWN)
                    return CellStatus::kUnknown;
                if ((thisPointProperties(p_n) == CellProperty::COLLIDE)
                    || (thisPointProperties(p_n) == CellProperty::COLLIDE_OPEN))
                    return CellStatus::kOccupied;
            }
            return CellStatus::kFree;
        }
    }

    CollisionDetector::CellStatus CollisionDetector::whatBetweenBoundingBox(const Eigen::Vector2d& start, const Eigen::Vector2d& end, const Eigen::Vector2d& bounding_box_size) {
        CellStatus ret;
        const Eigen::Vector2d bounding_box_half_size = bounding_box_size * 0.5;

        for (double x = -bounding_box_half_size.x(); x <= bounding_box_half_size.x(); x += resolution_) {
            for (double y = -bounding_box_half_size.y(); y <= bounding_box_half_size.y(); y += resolution_) {
                Eigen::Vector2d offset(x, y);
                ret = whatBetween(start + offset, end + offset);
                if ((ret == CellStatus::kOccupied) || (ret == CellStatus::kError)) {
                    return ret;
                }
            }
        }
        return CellStatus::kFree;
    }

    CollisionDetector::CellProperty CollisionDetector::thisPointProperties(const Eigen::Vector2d& point) {
        return thisPointProperties(point.x(), point.y());
    }

    CollisionDetector::CellProperty CollisionDetector::thisPointProperties(const Eigen::Vector2d& point, int8_t& cost) {
        return thisPointProperties(point.x(), point.y(), cost);
    }

    CollisionDetector::CellStatus CollisionDetector::thisPointStatus(const Eigen::Vector2d& point) {
        return thisPointStatus(point.x(), point.y());
    }

    CollisionDetector::CellStatus CollisionDetector::thisPointStatus(const Eigen::Vector2d& point, int8_t& cost) {
        return thisPointStatus(point.x(), point.y(), cost);
    }

    void CollisionDetector::setMap(nav_msgs::OccupancyGrid& costmap) {
        costmap_ = costmap;
        resolution_ = costmap_.info.resolution;
        size_x_ = costmap_.info.width;
        size_y_ = costmap_.info.height;
        origin_x_ = costmap_.info.origin.position.x;
        origin_y_ = costmap_.info.origin.position.y;
        data_ = costmap_.data;
    }

    void CollisionDetector::worldToMap(double wx, double wy, uint32_t& mx, uint32_t& my) {
            mx = static_cast<uint32_t>((wx - (double) origin_x_) / resolution_);
            my = static_cast<uint32_t>((wy - (double) origin_y_) / resolution_);
    }

    void CollisionDetector::mapToWorld(uint32_t mx, uint32_t my, double& wx, double& wy) {
            uint32_t indx = my * size_x_ + mx;
            wx = origin_x_ + indx % size_x_ * resolution_;
            wy = origin_y_ + indx / size_x_ * resolution_;
    }

    int8_t CollisionDetector::getGridValue(uint32_t mx, uint32_t my) {
        uint32_t indx = my * size_x_ + mx;
        return data_[indx];
    }

}  // namespace windPlanner
