#include "windplanner/rrt.hpp"


namespace windPlanner {

    RRT::RRT() {
        // Clear Tree
        bestGain_ = params_.zero_gain_;
        bestNode_ = nullptr;
        counter_ = 0;
        rootNode_ = nullptr;
        
        // Init Variables
        iterationCount_ = 0;
    }

    RRT::~RRT() {
        delete rootNode_;
    }

    void RRT::setStateFromPoseMsg(const geometry_msgs::PoseWithCovarianceStamped& pose) {
        // Get latest transform to the planning frame and transform the pose
        static tf::TransformListener listener;
        tf::StampedTransform transform;
        try {
            listener.waitForTransform(params_.mapFrame_, pose.header.frame_id, ros::Time(0), ros::Duration(10.0));
            listener.lookupTransform(params_.mapFrame_, pose.header.frame_id, ros::Time(0), transform);
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
            return;
        }

        tf::Pose poseTF;
        tf::poseMsgToTF(pose.pose.pose, poseTF);
        tf::Vector3 position = poseTF.getOrigin();
        position = transform * position;
        tf::Quaternion quat = poseTF.getRotation();
        quat = transform * quat;

        root_.x() = position.x();
        root_.y() = position.y();
        root_.z() = tf::getYaw(quat);
    }

    void RRT::setStateFromOdometryMsg(const nav_msgs::Odometry& pose) {
        // Get latest transform to the planning frame and transform the pose
        static tf::TransformListener listener;
        tf::StampedTransform transform;
        try {
            listener.waitForTransform(params_.mapFrame_, pose.header.frame_id, ros::Time(0), ros::Duration(10.0));
            listener.lookupTransform(params_.mapFrame_, pose.header.frame_id, ros::Time(0), transform);
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
            return;
        }

        tf::Pose poseTF;
        tf::poseMsgToTF(pose.pose.pose, poseTF);
        tf::Vector3 position = poseTF.getOrigin();
        position = transform * position;
        tf::Quaternion quat = poseTF.getRotation();
        quat = transform * quat;

        root_.x() = position.x();
        root_.y() = position.y();
        root_.z() = tf::getYaw(quat);
    }

    geometry_msgs::PoseWithCovarianceStamped RRT::setStateFromMapMsg(const nav_msgs::OccupancyGrid& msg) {
        costmap_ = msg;
        cd_.setMap(costmap_);
        resolution_ = costmap_.info.resolution;
        random_double_.setParams(costmap_.info.origin.position.x, costmap_.info.origin.position.y,
            costmap_.info.width, costmap_.info.height, costmap_.info.resolution);

        // Get latest transform to the planning frame and transform the pose
        static tf::TransformListener listener;
        tf::StampedTransform transform;
        try {
            listener.waitForTransform(params_.mapFrame_, params_.baseFrame_, ros::Time(0), ros::Duration(10.0));
            listener.lookupTransform(params_.mapFrame_, params_.baseFrame_, ros::Time(0), transform);
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
        }

        root_.x() = transform.getOrigin().x();
        root_.y() = transform.getOrigin().y();
        root_.z() = tf::getYaw(transform.getRotation());

        if (costmap_.data.size() > mapWeight_.size()) {
            isNeedWeightSampling_ = false;
            weightMap_ = costmap_;
            mapWeight_.clear();
            mapWeight_.resize(costmap_.data.size());
            for (std::size_t i = 0; i < mapWeight_.size(); i++) {
                mapWeight_[i] = 1.0;
            }
        }
        else {
            isNeedWeightSampling_ = true;
            slideMapWeight();
            weight_ = calculateWeight();
        }

        geometry_msgs::PoseWithCovarianceStamped robPos;
        robPos.header.frame_id = "base_link";
        robPos.header.stamp = ros::Time::now();
        robPos.pose.pose.position.x = root_.x();
        robPos.pose.pose.position.y = root_.y();
        robPos.pose.pose.position.z = 0.0;
        tf::quaternionTFToMsg(transform.getRotation(), robPos.pose.pose.orientation);
        
        visualizeWeight();
        return robPos;
    }

    void RRT::initialize() {
        // This function is to initialize the tree, including insertion of remainder of previous best branch.
        rootNode_ = new Node;
        if (params_.exact_root_) {
            if (iterationCount_ <= 1)
                exact_root_ = root_;
            rootNode_->state = exact_root_;
        }
        else
            rootNode_->state = root_;

        rootNode_->node_id = 0;
        rootNode_->distance = 0.0;
        rootNode_->gain = params_.zero_gain_;
        rootNode_->parent = nullptr;
        rootNode_->children.push_back(nullptr);

        iterationCount_++;

        nodes_.push_back(rootNode_);
        publishTreePoints(rootNode_->state);

        // Insert all nodes of the remainder of the previous best branch, checking for collisions and
        // recomputing the gain.
        for (std::vector<Eigen::Vector3d>::reverse_iterator iter = bestBranchMemory_.rbegin();
        iter != bestBranchMemory_.rend(); ++iter) {
            Eigen::Vector3d newState = *iter;
            Node* newParent = getNearestNode(newState);

            // Check for collision
            Eigen::Vector2d origin(newParent->state.x(), newParent->state.y());
            Eigen::Vector2d direction(newState.x() - newParent->state.x(), newState.y() - newParent->state.y());
            
            if (direction.norm() > params_.extensionRange_) {
                direction = params_.extensionRange_ * direction.normalized();
            }

            newState.x() = origin.x() + direction.x();
            newState.y() = origin.y() + direction.y();
            newState.z() = atan2(direction.y(), direction.x());

            if (CollisionDetector::CellStatus::kFree
            != cd_.thisPointStatusBoundingBox(Eigen::Vector2d(newState.x(), newState.y()), params_.boundingBox_)) {
                continue;
            }
            if (direction.norm() < params_.minExtensionRange_) {
                continue;
            }
            counter_++;

            Eigen::Vector2d overshoot = origin + direction + direction.normalized() * params_.dOvershoot_;

            bool checkVisibility = (CollisionDetector::CellStatus::kFree == cd_.whatBetweenBoundingBox(origin, overshoot, params_.boundingBox_));
            if (checkVisibility) {
                // Create new node and insert into tree
                Node* newNode = new Node;
                newNode->state = newState;
                newNode->parent = newParent;
                newNode->distance = newParent->distance + direction.norm();
                newParent->children.push_back(newNode);
                newNode->node_id = nodes_.size();
                newNode->gain = newParent->gain
                    + computeGain(newNode->state) * exp(-params_.degressiveCoeff_ * newNode->distance);

                nodes_.push_back(newNode);

                // Display new node
                publishNode(newNode);

                // Update best IG and node if applicable
                if (newNode->gain > bestGain_) {
                    bestGain_ = newNode->gain;
                    bestNode_ = newNode;
                }
            }
        }
    }

    void RRT::iterate() {
        // In this function a new configuration is sampled and added to the tree.
        Eigen::Vector3d newState;
        bool solutionFound_pos = false;

        Eigen::Vector2d origin, direction(0.0, 0.0);
        Node* newParent;

        // Find nearest neighbour
        while ((!solutionFound_pos)) {
            newState = sampleFree();
            newParent = getNearestNode(newState);

            // Check for collision of new connection plus some overshoot distance.
            origin = Eigen::Vector2d(newParent->state.x(), newParent->state.y());
            direction = Eigen::Vector2d(newState.x() - newParent->state.x(), newState.y() - newParent->state.y());

            if (direction.norm() > params_.extensionRange_) {
                direction = params_.extensionRange_ * direction.normalized();
            }

            newState.x() = origin.x() + direction.x();
            newState.y() = origin.y() + direction.y();
            newState.z() = atan2(direction.y(), direction.x());

            bool checkPoint = CollisionDetector::CellStatus::kFree
            != cd_.thisPointStatusBoundingBox(Eigen::Vector2d(newState.x(), newState.y()), params_.boundingBox_);
            // && CollisionDetector::CellStatus::kUnknown
            // != cd_.thisPointStatusBoundingBox(Eigen::Vector2d(newState.x(), newState.y()), params_.boundingBox_);

            if (checkPoint) {
                continue;
            }
            if (direction.norm() < params_.minExtensionRange_) {
                continue;
            }
            solutionFound_pos = true;
            counter_++;
        }
        
        Eigen::Vector2d overshoot = origin + direction + direction.normalized() * params_.dOvershoot_;

        bool checkVisibility = (CollisionDetector::CellStatus::kFree == cd_.whatBetweenBoundingBox(origin, overshoot, params_.boundingBox_));
        if (checkVisibility) {
            // Create new node and insert into tree
            Node* newNode = new Node;
            newNode->state = newState;
            newNode->parent = newParent;
            newNode->distance = newParent->distance + direction.norm();
            newParent->children.push_back(newNode);
            newNode->node_id = nodes_.size();
            newNode->gain = newParent->gain
                + computeGain(newNode->state) * exp(-params_.degressiveCoeff_ * newNode->distance);

            nodes_.push_back(newNode);

            // Display new node
            publishNode(newNode);

            // Update best IG and node if applicable
            if (newNode->gain > bestGain_) {
                bestGain_ = newNode->gain;
                bestNode_ = newNode;
            }
        }
    }

    double RRT::computeGain(Eigen::Vector3d state) {
        // This function computes the gain
        double gain = 0.0;
        double unmapped_gain = 0.0;
        double occupied_gain = 0.0;
        double free_gain = 0.0;
        double occupancy_thres = 0.5;
        double occupancy_scale = 0.1;

        double clamping_thres_min = 0.12;
        double clamping_thres_max = 0.97;
        // double occupancy_thres = 0.7;
        const double occupied_occupancy_scale = clamping_thres_max - occupancy_thres;
        const double free_occupancy_scale = occupancy_thres - clamping_thres_min;
        
        Eigen::Vector2d origin(state.x(), state.y());
        Eigen::Vector2d vec;
        double rangeSq = pow(params_.gainRange_, 2.0);
        
        // Iterate over all nodes within the allowed distance
        for (std::size_t i = 0; i < params_.numberSamples_; i++) {
            vec.x() = state.x() + static_cast<double>((((double) rand()) / ((double) RAND_MAX) - 0.5) * params_.gainRange_ * 2.0);
            vec.y() = state.y() + static_cast<double>((((double) rand()) / ((double) RAND_MAX) - 0.5) * params_.gainRange_ * 2.0);
            Eigen::Vector2d dir = vec - origin;
            
            // Skip if distance is too large
            if (dir.transpose().dot(dir) > rangeSq) {
                continue;
            }
            
            // Check cell status and add to the gain considering the corresponding factor.
            int8_t probability;
            CollisionDetector::CellStatus nodeStatus = cd_.thisPointStatus(vec.x(), vec.y(), probability);

            bool checkVisibility = (CollisionDetector::CellStatus::kOccupied != cd_.whatBetween(origin, vec))
                && (CollisionDetector::CellStatus::kError != cd_.whatBetween(origin, vec));
            if (nodeStatus == CollisionDetector::CellStatus::kError) {
                continue;
            }
            if (nodeStatus == CollisionDetector::CellStatus::kUnknown) {
                if (checkVisibility) {
                    unmapped_gain += pow(resolution_, 2.0) * params_.igUnmapped_;
                }
            }
            if (nodeStatus == CollisionDetector::CellStatus::kOccupied) {
                if (checkVisibility) {
                    // occupied_gain += pow(resolution_, 2.0) * params_.igOccupied_;
                    // TODO: Add probabilistic gain
                    occupied_gain += pow(resolution_, 2.0) * params_.igOccupied_ * params_.igProbabilistic_
                        * exp(-0.5 * pow(((double) probability / 100.0 - occupancy_thres) / occupancy_scale, 2.0)) / sqrt(2 * M_PI * pow(occupancy_scale, 2.0));
                    // occupied_gain += pow(resolution_, 2.0) * params_.igOccupied_ * (params_.igProbabilistic_ / occupied_occupancy_scale) * (clamping_thres_max - (double) probability / 100.0);
                }
            }
            else {
                if (checkVisibility) {
                    // free_gain += pow(resolution_, 2.0) * params_.igFree_;
                    // TODO: Add probabilistic gain
                    free_gain += pow(resolution_, 2.0) * params_.igFree_ * params_.igProbabilistic_
                        * exp(-0.5 * pow(((double) probability / 100.0 - occupancy_thres) / occupancy_scale, 2.0)) / sqrt(2 * M_PI * pow(occupancy_scale, 2.0));
                    // free_gain += pow(resolution_, 2.0) * params_.igFree_ * (params_.igProbabilistic_ / free_occupancy_scale) * (1.0 - (double) probability / 100.0 - clamping_thres_min);
                }
            }
        }

        gain = unmapped_gain + occupied_gain + free_gain;
        return gain;
    }

    geometry_msgs::Pose RRT::sampleGoal(Eigen::Vector3d state, std::string targetFrame) {
        geometry_msgs::Pose ret;
        static tf::TransformListener listener;
        tf::StampedTransform transform;
        try {
            listener.waitForTransform(params_.mapFrame_, targetFrame, ros::Time(0), ros::Duration(10.0));
            listener.lookupTransform(params_.mapFrame_, targetFrame, ros::Time(0), transform);
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
            return ret;
        }
        
        double yaw = state.z();
        if (yaw > M_PI) {
            yaw -= 2.0 * M_PI;
        }
        if (yaw < -M_PI) {
            yaw += 2.0 * M_PI;
        }

        tf::Vector3 origin(state.x(), state.y(), 0.0);

        tf::Quaternion quat;
        quat.setEuler(0.0, 0.0, yaw);
        origin = transform * origin;
        quat = transform * quat;
        tf::Pose poseTF(quat, origin);
        tf::poseTFToMsg(poseTF, ret);
        
        return ret;
    }

    void RRT::slideMapWeight() {
        Eigen::Vector2d rootTemp(root_.x(), root_.y());
        Eigen::Vector2d bbx_min_eigen = rootTemp - params_.mapWeightBox_ / 2;
        Eigen::Vector2d bbx_max_eigen = rootTemp + params_.mapWeightBox_ / 2;

        for (double wx = bbx_min_eigen.x(); wx <= bbx_max_eigen.x(); wx += resolution_) {
            for (double wy = bbx_min_eigen.y(); wy <= bbx_max_eigen.y(); wy += resolution_) {
                uint32_t mx = static_cast<uint32_t>((wx - (double) weightMap_.info.origin.position.x) / resolution_);
                uint32_t my = static_cast<uint32_t>((wy - (double) weightMap_.info.origin.position.x) / resolution_);
                uint32_t indx = my * weightMap_.info.width + mx;
                if (indx < mapWeight_.size() && indx > 0) mapWeight_[indx] += 1;
            }
        }
    }

    Eigen::Vector3d RRT::sampleFree() {
        Eigen::Vector2d random_x_y;

        if (isNeedWeightSampling_) {
            random_x_y = random_double_.generate(weight_);
        }
        else {
            random_x_y = random_double_.generate();
        }
        Eigen::Vector3d random_point;
        random_point.x() = random_x_y.x();
        random_point.y() = random_x_y.y();    
        random_point.z() = atan2(random_x_y.x(), random_x_y.y());
        
        return random_point;
    }

    std::vector<double> RRT::calculateWeight() {
        std::vector<double> weight = std::vector<double>(mapWeight_.begin(), mapWeight_.end());

        for (std::size_t i = 0; i < weight.size(); i++) {
            weight[i] = 1 / weight[i];
        }
        
        double sum = std::accumulate(weight.begin(), weight.end(), decltype(weight)::value_type(0));
        for (std::size_t i = 0; i < weight.size(); i++) {
            weight[i] = weight[i] / sum;
        }

        return weight;
    }

    void RRT::visualizeWeight() {
        weightMap_.header.frame_id = "weight";
        for (std::size_t i = 0; i < weight_.size(); i++) {
            weight_[i] = weight_[i] * 100.0;
        }
        weightMap_.data = std::vector<int8_t>(weight_.begin(), weight_.end());
    }

    Node* RRT::getNearestNode(Eigen::Vector3d state) {
        double dist_nearest, dist;
        std::size_t nearest_id = 0;
        for (std::size_t i = 1; i < nodes_.size(); ++i) {
            dist_nearest = euclideanDistance2D(nodes_[nearest_id]->state.x(), nodes_[nearest_id]->state.y(), state.x(), state.y());
            dist = euclideanDistance2D(nodes_[i]->state.x(), nodes_[i]->state.y(), state.x(), state.y());
            if (dist < dist_nearest)
                nearest_id = i;
        }

        return nodes_[nearest_id];
    }

    void RRT::clear() {
        delete rootNode_;
        rootNode_ = nullptr;

        counter_ = 0;
        bestGain_ = params_.zero_gain_;
        delete bestNode_;
        bestNode_ = nullptr;

        while (nodes_.size() > 0) {
            nodes_.pop_back();
        }
    }

    void RRT::memorizeBestBranch() {
        bestBranchMemory_.clear();
        Node* current = bestNode_;
        while (current->parent && current->parent->parent) {
            bestBranchMemory_.push_back(current->state);
            current = current->parent;
        }
    }

    std::vector<geometry_msgs::Pose> RRT::getBestEdge(std::string targetFrame) {
        // This function returns the first edge of the best branch
        std::vector<geometry_msgs::Pose> ret;
        Node* current = bestNode_;
        if (current->parent != nullptr) {
            while (current->parent != nullptr) {
                publishTreePoints(current->state);
                ret.push_back(sampleGoal(current->state, targetFrame));
                history_.push(current->parent->state);
                current = current->parent;
            }
            exact_root_ = current->state;
            ret.push_back(sampleGoal(exact_root_, targetFrame));
        }
        return ret;
    }

    std::vector<geometry_msgs::Pose> RRT::getPathBackToPrevious(std::string targetFrame) {
        std::vector<geometry_msgs::Pose> ret;
        if (history_.empty()) {
            return ret;
        }
        ret.push_back(sampleGoal(history_.top(), targetFrame));
        history_.pop();
        return ret;
    }

    void RRT::publishNode(Node* node) {
        // params_.visualizeTrees_->publishSphere(Eigen::Vector3d(node->state.x(), node->state.y(), 0.0),
        // rviz_visual_tools::YELLOW, rviz_visual_tools::XXXLARGE, "TreeEndPoints");

        if (!node->parent)
            return;
        
        geometry_msgs::Point start, end;
        start.x = node->parent->state.x();
        start.y = node->parent->state.y();
        start.z = 0.0;
        end.x = node->state.x();
        end.y = node->state.y();
        end.z = 0.0;

        params_.visualizeTrees_->publishArrow(start, end, rviz_visual_tools::YELLOW, rviz_visual_tools::LARGE);
    }

    void RRT::publishTreePoints(Eigen::Vector3d state) {
        params_.visualizeTreePoints_->publishSphere(Eigen::Vector3d(state.x(), state.y(), 0.0),
        rviz_visual_tools::RED, rviz_visual_tools::XXXLARGE, "TreeSamplePoints");
    }

    std::vector<Node*> RRT::getNodes() const {
        return nodes_;
    }

    nav_msgs::OccupancyGrid RRT::getWeightMap() {
        return weightMap_;
    }

    void RRT::setParams(Params &params) {
        params_ = params;
    }

    uint32_t RRT::getCounter() {
        return counter_;
    }

    bool RRT::gainFound() {
        return bestGain_ > params_.zero_gain_;
    }
}  // namespace windPlanner
