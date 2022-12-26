#ifndef NODE_HPP_  // NOLINT
#define NODE_HPP_

#include <Eigen/Dense>
#include <cmath>

namespace windPlanner {

    inline double euclideanDistance2D(double x1, double y1, double x2, double y2) {
        return std::hypot((x1 - x2), (y1 - y2));
    }

    inline double euclideanDistance2D(Eigen::Vector2d start, Eigen::Vector2d end) {
        return euclideanDistance2D(start.x(), start.y(), end.x(), end.y());
    }

    struct Node {
        Eigen::Vector3d state;
        uint32_t node_id;
        Node* parent;
        std::vector<Node*> children;
        double distance{0.0};
        double gain{0.0};

        Node() {}
        ~Node() {}

        Node(double px, double py, double yaw, uint32_t node_index) : state(Eigen::Vector3d(px, py, yaw)), node_id(node_index) {}

        Node(double px, double py, double yaw, uint32_t node_index,
            double pdistance, double pgain) : state(Eigen::Vector3d(px, py, yaw)),
                                        node_id(node_index),
                                        distance(pdistance),
                                        gain(pgain) {}

        bool operator ==(const Node& node) { return node_id == node.node_id; }

        bool operator !=(const Node& node) { return !(node_id == node.node_id); }
    };

}  // namespace windPlanner

#endif  // NODE_HPP_  NOLINT
