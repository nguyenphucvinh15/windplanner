#include "windplanner/pe.hpp"


namespace windPlanner {
    
    std::vector<geometry_msgs::Pose> PE::initialize() {
        x_est_.clear();
        vec_px_.clear();
        weight_.clear();
        xOrigins_.clear();
        landMarks_.clear();

        x_est_.resize(reqPath_.size());
        vec_px_.resize(reqPath_.size());
        weight_.resize(reqPath_.size());
        xOrigins_.resize(reqPath_.size());
        landMarks_.resize(reqPath_.size());
                                      
        params_.visualizeLandMarks_->deleteAllMarkers();

        for (goalLoc_ = 0; goalLoc_ < reqPath_.size(); goalLoc_++) {
            xOrigins_.at(goalLoc_) = Eigen::Vector2d(reqPath_.at(goalLoc_).position.x, reqPath_.at(goalLoc_).position.y);
            x_est_.at(goalLoc_) = xOrigins_.at(goalLoc_);
            
            std::vector<double> w(params_.numberSamples, 1.0 / (double) params_.numberSamples);
            weight_.at(goalLoc_) = w;

            std::normal_distribution<double> dist_x(reqPath_.at(goalLoc_).position.x, params_.sampleRange);
            std::normal_distribution<double> dist_y(reqPath_.at(goalLoc_).position.y, params_.sampleRange);
            std::default_random_engine gen;

            while (vec_px_.at(goalLoc_).size() < params_.numberSamples) {
                Eigen::Vector2d vec_px;
                vec_px.x() = dist_x(gen);
                vec_px.y() = dist_y(gen);

                CollisionDetector::CellStatus nodeStatus = cd_.thisPointStatusBoundingBox(vec_px, params_.boundingBox);
                if (CollisionDetector::CellStatus::kFree == nodeStatus) {
                    vec_px_.at(goalLoc_).push_back(vec_px);

                    // Visualize Samples
                    publishSamples(vec_px);
                }
                else {
                    continue;
                }
            }
            landMarks_.at(goalLoc_) = getLandMarks();
        }

        // Trigger Visualize All Samples
        params_.visualizeLandMarks_->trigger();

        return reqPath_;
    }

    std::vector<geometry_msgs::Pose> PE::iterate() {
        // Delete All Previous Markers
        params_.visualizeEstimates_->deleteAllMarkers();
        params_.visualizeSamples_->deleteAllMarkers();
        params_.visualizeReWeightedSamples_->deleteAllMarkers();

        std::vector<geometry_msgs::Pose> resPath;
        for (goalLoc_ = 0; goalLoc_ < reqPath_.size(); goalLoc_++) {
            xMeas_ = Eigen::Vector2d(reqPath_.at(goalLoc_).position.x, reqPath_.at(goalLoc_).position.y);
            std::vector<double> weight = predictEstimate(weight_.at(goalLoc_));

            geometry_msgs::Pose estimate = reSampling(weight);
            resPath.push_back(estimate);
        }

        publishEstTrees();

        // Visualize All Estimate Goals
        params_.visualizeEstimates_->trigger();

        // Visualize All Resampling Points
        params_.visualizeSamples_->trigger();
        params_.visualizeReWeightedSamples_->trigger();

        return resPath;
    }

    Eigen::Vector2d PE::getLandMarks() {    
        Eigen::Vector2d px, landMark;
        std::vector<Eigen::Vector2d> landMarks;
        std::vector<double> rAs, rProbs;

        std::normal_distribution<double> dist_x(xOrigins_.at(goalLoc_).x(), params_.replanRange);
        std::normal_distribution<double> dist_y(xOrigins_.at(goalLoc_).y(), params_.replanRange);
        std::default_random_engine genUnknown, genFree;

        for (std::size_t i = 0; i < params_.numberSamples; i++) {
            px.x() = dist_x(genUnknown);
            px.y() = dist_y(genUnknown);

            CollisionDetector::CellStatus nodeStatus = cd_.thisPointStatusBoundingBox(px, params_.boundingBox);
            
            if (CollisionDetector::CellStatus::kUnknown == nodeStatus) {
                bool checkVisibility = (CollisionDetector::CellStatus::kFree == cd_.whatBetweenBoundingBox(xOrigins_.at(goalLoc_), px, params_.boundingBox));
                if (checkVisibility) {
                    landMarks.push_back(px);
                    rAs.push_back(euclideanDistance2D(px, xOrigins_.at(goalLoc_)));
                }
            }
        }

        if (rAs.size() == 0) {
            for (std::size_t i = 0; i < params_.numberSamples; i++) {
                px.x() = dist_x(genFree);
                px.y() = dist_y(genFree);

                int8_t probability;
                CollisionDetector::CellStatus nodeStatus = cd_.thisPointStatusBoundingBox(px, params_.boundingBox, probability);
                
                if (CollisionDetector::CellStatus::kFree == nodeStatus) {
                    bool checkVisibility = (CollisionDetector::CellStatus::kFree == cd_.whatBetweenBoundingBox(xOrigins_.at(goalLoc_), px, params_.boundingBox));
                    if (checkVisibility) {
                        landMarks.push_back(px);
                        rProbs.push_back(probability);
                        rAs.push_back(euclideanDistance2D(px, xOrigins_.at(goalLoc_)));
                    }
                }
            }
        
            double rProb = *max_element(rProbs.begin(), rProbs.end());
            auto rIdx = std::max_element(rProbs.begin(), rProbs.end()) - rProbs.begin();
            landMark = landMarks.at(rIdx);
            rA_ = rAs.at(rIdx);
        }
        else {
            rA_ = *std::min_element(rAs.begin(), rAs.end());
            auto rIdx = std::min_element(rAs.begin(), rAs.end()) - rAs.begin();
            landMark = landMarks.at(rIdx);
        }

        // Visualize Landmarks
        publishLandMarks(landMark);

        return landMark;
    }

    std::vector<double> PE::predictEstimate(std::vector<double> weight) {
        std::vector<double> rS;
        for (std::size_t i = 0; i < params_.numberSamples; i++) {
            double rsamp = euclideanDistance2D(vec_px_.at(goalLoc_).at(i), landMarks_.at(goalLoc_));

            weight.at(i) *= exp(-0.5 * pow(rsamp, 2.0) / R_) / sqrt(2 * M_PI * R_);
        }

        double sum = std::accumulate(weight.begin(), weight.end(), decltype(weight)::value_type(0));
        if (sum == 0) {
            ROS_WARN_THROTTLE(1, "Problem with Random Samples Weight for Replanning! Retry...");

            std::vector<double> w(params_.numberSamples, 1.0 / (double) params_.numberSamples);
            weight = w;
            vec_px_.at(goalLoc_).clear();

            std::normal_distribution<double> dist_x(reqPath_.at(goalLoc_).position.x, params_.sampleRange);
            std::normal_distribution<double> dist_y(reqPath_.at(goalLoc_).position.y, params_.sampleRange);
            std::default_random_engine gen;

            while (vec_px_.at(goalLoc_).size() < params_.numberSamples) {
                Eigen::Vector2d vec_px;
                vec_px.x() = dist_x(gen);
                vec_px.y() = dist_y(gen);

                CollisionDetector::CellStatus nodeStatus = cd_.thisPointStatusBoundingBox(vec_px, params_.boundingBox);
                if (CollisionDetector::CellStatus::kFree == nodeStatus) {
                    vec_px_.at(goalLoc_).push_back(vec_px);

                    // publishSamples(vec_px);
                }
                else {
                    continue;
                }
            }
        }

        for (std::size_t i = 0; i < params_.numberSamples; i++) {
            weight.at(i) = weight.at(i) / sum;
        }

        x_est_.at(goalLoc_) = calculateEstimate(vec_px_.at(goalLoc_), weight);

        // Visualize Estimate Goals
        // publishEstimates(x_est_.at(goalLoc_));

        return weight;
    }

    geometry_msgs::Pose PE::reSampling(std::vector<double> weight) {
        // Resampling
        std::discrete_distribution<std::size_t> distribution(weight.begin(), weight.end());
        std::default_random_engine gen;

        std::vector<Eigen::Vector2d> resampled_particles;
        std::vector<double> resampled_weight;

        for (std::size_t i = 0; i < params_.numberSamples; i++){
            auto distributeIndex = distribution(gen);
            resampled_particles.push_back(vec_px_.at(goalLoc_).at(distributeIndex));
            resampled_weight.push_back(weight.at(distributeIndex));

            // Visualize Resampling Points
            publishReWeightedSamples(vec_px_.at(goalLoc_).at(distributeIndex));
        }

        vec_px_.at(goalLoc_) = resampled_particles;
        weight_.at(goalLoc_) = resampled_weight;

        geometry_msgs::Pose estimate;
        if (CollisionDetector::CellStatus::kOccupied != cd_.thisPointStatusBoundingBox(x_est_.at(goalLoc_), params_.boundingBox)) {
            estimate.position.x = x_est_.at(goalLoc_).x();
            estimate.position.y = x_est_.at(goalLoc_).y();
            estimate.position.z = reqPath_.at(goalLoc_).position.z;
            estimate.orientation = reqPath_.at(goalLoc_).orientation;
        }
        else {
            ROS_WARN("Cannot Replan new Goal! Skipping and Resampling Again!");
            estimate.position.x = xMeas_.x();
            estimate.position.y = xMeas_.y();
            estimate.position.z = reqPath_.at(goalLoc_).position.z;
            estimate.orientation = reqPath_.at(goalLoc_).orientation;

            std::vector<double> w(params_.numberSamples, 1.0 / (double) params_.numberSamples);
            weight_.at(goalLoc_) = w;
            vec_px_.at(goalLoc_).clear();

            std::normal_distribution<double> dist_x(xMeas_.x(), params_.sampleRange);
            std::normal_distribution<double> dist_y(xMeas_.y(), params_.sampleRange);
            std::default_random_engine gen_;

            while (vec_px_.at(goalLoc_).size() < params_.numberSamples) {
                Eigen::Vector2d vec_px;
                vec_px.x() = dist_x(gen_);
                vec_px.y() = dist_y(gen_);

                CollisionDetector::CellStatus nodeStatus = cd_.thisPointStatusBoundingBox(vec_px, params_.boundingBox);
                if (CollisionDetector::CellStatus::kFree == nodeStatus) {
                    vec_px_.at(goalLoc_).push_back(vec_px);
                }
                else {
                    continue;
                }
            }
        }

        return estimate;
    }

    Eigen::Vector2d PE::calculateEstimate(std::vector<Eigen::Vector2d> matA, std::vector<double> matB) {
        Eigen::Vector2d result(0.0, 0.0);
        for (std::size_t i = 0; i < matB.size(); i++) {
            result.x() += matA.at(i).x() * matB.at(i);
            result.y() += matA.at(i).y() * matB.at(i);
        }

        return result;
    }

    void PE::setParams(Params &params) {
        params_ = params;
        R_ = pow(params.sampleRange, 2);
    }

    void PE::setPaths(std::vector<geometry_msgs::Pose> reqPath) {
        reqPath_ = reqPath;
    }

    void PE::setMap(const nav_msgs::OccupancyGrid& msg) {
        map_ = msg;
        cd_.setMap(map_);
    }

    void PE::publishEstimates(Eigen::Vector2d state) {
        params_.visualizeEstimates_->publishSphere(Eigen::Vector3d(state.x(), state.y(), 0.0), rviz_visual_tools::BLUE, rviz_visual_tools::XXXLARGE, "Estimates");    
    }

    void PE::publishLandMarks(Eigen::Vector2d state) {
        params_.visualizeLandMarks_->publishSphere(Eigen::Vector3d(state.x(), state.y(), 0.0), rviz_visual_tools::GREEN, rviz_visual_tools::XXXLARGE, "Landmarks");    
    }

    void PE::publishSamples(Eigen::Vector2d state) {
        params_.visualizeSamples_->publishSphere(Eigen::Vector3d(state.x(), state.y(), 0.0), rviz_visual_tools::GREY, rviz_visual_tools::XXXLARGE, "Samples");    
    }

    void PE::publishReWeightedSamples(Eigen::Vector2d state) {
        params_.visualizeReWeightedSamples_->publishSphere(Eigen::Vector3d(state.x(), state.y(), 0.0), rviz_visual_tools::BROWN, rviz_visual_tools::XXXLARGE, "Resamples");    
    }

    void PE::publishEstTrees() {
        for (std::vector<Eigen::Vector2d>::iterator loc = x_est_.begin(); loc != x_est_.end(); ++loc) {
            Eigen::Vector2d start = *loc;
            Eigen::Vector2d end = *(loc + 1);
            geometry_msgs::Point startPoint, endPoint;
            startPoint.x = start.x();
            startPoint.y = start.y();
            startPoint.z = 0.0;
            endPoint.x = end.x();
            endPoint.y = end.y();
            endPoint.z = 0.0;

            publishEstimates(start);
        }
    }

}