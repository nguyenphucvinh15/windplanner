#include "windplanner/rhe.hpp"


namespace windPlanner {

    Planner::Planner(const ros::NodeHandle& nh) : nh_(nh) {
        plannerService_ = nh_.advertiseService("rheplanner", &Planner::plannerCallback, this);
        rePlannerService_ = nh_.advertiseService("rhereplanner", &Planner::rePlannerCallback, this);

        if (!setParams()) {
            ROS_ERROR("Could not start the planner. Parameters missing!");
        }

        // posClient_ = nh_.subscribe(params_.poseTopic_, 10, &Planner::posCallback, this);
        // odomClient_ = nh_.subscribe(params_.odomTopic_, 10, &Planner::odomCallback, this);
        mapClient_ = nh_.subscribe(params_.mapTopic_, 10, &Planner::mapCallback, this);
        
        weightMapPub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/weightMap", 1000);
        robPosPub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/pose", 1000);
        pathPub_ = nh_.advertise<geometry_msgs::PoseArray>("/computedPath", 1000);

        params_.visualizeTrees_.reset(new rviz_visual_tools::RvizVisualTools("map", "/visualizeTrees"));
        params_.visualizeTreePoints_.reset(new rviz_visual_tools::RvizVisualTools("map", "/visualizeTreePoints"));
        paramsRePlanner_.visualizeEstimates_.reset(new rviz_visual_tools::RvizVisualTools("map", "/visualizeEstimates"));
        paramsRePlanner_.visualizeLandMarks_.reset(new rviz_visual_tools::RvizVisualTools("map", "/visualizeLandMasks"));
        paramsRePlanner_.visualizeSamples_.reset(new rviz_visual_tools::RvizVisualTools("map", "/visualizeSamples"));
        paramsRePlanner_.visualizeReWeightedSamples_.reset(new rviz_visual_tools::RvizVisualTools("map", "/visualizeReWeightedSamples"));
        
        params_.visualizeTrees_->loadMarkerPub();
        params_.visualizeTreePoints_->loadMarkerPub();
        paramsRePlanner_.visualizeEstimates_->loadMarkerPub();
        paramsRePlanner_.visualizeLandMarks_->loadMarkerPub();
        paramsRePlanner_.visualizeSamples_->loadMarkerPub();
        paramsRePlanner_.visualizeReWeightedSamples_->loadMarkerPub();

        params_.visualizeTrees_->enableBatchPublishing();
        params_.visualizeTreePoints_->enableBatchPublishing();
        paramsRePlanner_.visualizeEstimates_->enableBatchPublishing();
        paramsRePlanner_.visualizeLandMarks_->enableBatchPublishing();
        paramsRePlanner_.visualizeSamples_->enableBatchPublishing();
        paramsRePlanner_.visualizeReWeightedSamples_->enableBatchPublishing();

        // Initialize the tree instance.
        tree_ = new RRT();
        rePlanner_ = new PE();

        tree_->setParams(params_);
        rePlanner_->setParams(paramsRePlanner_);

        // Not yet ready. Needs messages first.
        initialized_ = false;

        try {
            listener_.waitForTransform(params_.mapFrame_, params_.baseFrame_, ros::Time(0), ros::Duration(10.0));
            
            // Planner is now ready to plan.
            initialized_ = true;
        }
        catch (tf::TransformException ex) {
            ROS_ERROR_STREAM("Error getting TF transform from sensor data: " << ex.what());
            ros::Duration(1.0).sleep();
        }
    }

    bool Planner::setParams() {
        std::string ns = ros::this_node::getName();
        bool ret = true;

        params_.mapFrame_ = "map";
        if (!ros::param::get(ns + "/system/map_frame", params_.mapFrame_)) {
            ROS_WARN("No map frame specified. Looking for %s. Default is 'map'.",
                    (ns + "/system/map_frame").c_str());
        }
        params_.baseFrame_ = "base_link";
        if (!ros::param::get(ns + "/system/robot_frame", params_.baseFrame_)) {
            ROS_WARN("No base frame of Robot specified. Looking for %s. Default is 'base_link'.",
                    (ns + "/system/robot_frame").c_str());
        }
        params_.poseTopic_ = "/pose";
        if (!ros::param::get(ns + "/system/pose_topic", params_.poseTopic_)) {
            ROS_WARN("No pose topic specified. Looking for %s. Default is '/pose'.",
                    (ns + "/system/pose_topic").c_str());
        }
        params_.odomTopic_ = "/odom";
        if (!ros::param::get(ns + "/system/odom_topic", params_.odomTopic_)) {
            ROS_WARN("No odometry topic specified. Looking for %s. Default is '/odom'.",
                    (ns + "/system/odom_topic").c_str());
        }
        params_.mapTopic_ = "/map";
        if (!ros::param::get(ns + "/system/map_topic", params_.mapTopic_)) {
            ROS_WARN("No map topic specified. Looking for %s. Default is '/map'.",
                    (ns + "/system/map_topic").c_str());
        }
        params_.dOvershoot_ = 0.5;
        if (!ros::param::get(ns + "/nbvpe/gain/overshoot", params_.dOvershoot_)) {
            ROS_WARN("No estimated overshoot value for collision avoidance specified. Looking for %s. Default is 0.5m.",
                (ns + "/nbvpe/gain/overshoot").c_str());
        }
        params_.igProbabilistic_ = 0.0;
        if (!ros::param::get(ns + "/nbvpe/gain/probabilistic", params_.igProbabilistic_)) {
            ROS_WARN("No gain coefficient for probability of cells specified. Looking for %s. Default is 0.0.",
                    (ns + "/nbvpe/gain/probabilistic").c_str());
        }
        params_.igFree_ = 0.0;
        if (!ros::param::get(ns + "/nbvpe/gain/free", params_.igFree_)) {
            ROS_WARN("No gain coefficient for free cells specified. Looking for %s. Default is 0.0.",
                    (ns + "/nbvpe/gain/free").c_str());
        }
        params_.igOccupied_ = 0.0;
        if (!ros::param::get(ns + "/nbvpe/gain/occupied", params_.igOccupied_)) {
            ROS_WARN("No gain coefficient for occupied cells specified. Looking for %s. Default is 0.0.",
                    (ns + "/nbvpe/gain/occupied").c_str());
        }
        params_.igUnmapped_ = 1.0;
        if (!ros::param::get(ns + "/nbvpe/gain/unmapped", params_.igUnmapped_)) {
            ROS_WARN("No gain coefficient for unmapped cells specified. Looking for %s. Default is 1.0.",
                    (ns + "/nbvpe/gain/unmapped").c_str());
        }
        params_.gainRange_ = 1.0;
        if (!ros::param::get(ns + "/nbvpe/gain/range", params_.gainRange_)) {
            ROS_WARN("No gain range specified. Looking for %s. Default is 1.0m.",
                    (ns + "/nbvpe/gain/range").c_str());
        }
        params_.degressiveCoeff_ = 0.25;
        if (!ros::param::get(ns + "/nbvpe/gain/degressive_coeff", params_.degressiveCoeff_)) {
            ROS_WARN("No degressive factor for gain accumulation specified. Looking for %s. Default is 0.25.",
                    (ns + "/nbvpe/gain/degressive_coeff").c_str());
        }
        params_.zero_gain_ = 0.0;
        if (!ros::param::get(ns + "/nbvpe/gain/zero", params_.zero_gain_)) {
            ROS_WARN("No zero gain value specified. Looking for %s. Default is 0.0.",
                    (ns + "/nbvpe/gain/zero").c_str());
        }
        params_.extensionRange_ = 1.0;
        if (!ros::param::get(ns + "/nbvpe/tree/extension_range", params_.extensionRange_)) {
            ROS_WARN("No value for maximal extension range specified. Looking for %s. Default is 1.0m.",
                    (ns + "/nbvpe/tree/extension_range").c_str());
        }
        params_.minExtensionRange_ = 1.0;
        if (!ros::param::get(ns + "/nbvpe/tree/min_extension_range", params_.minExtensionRange_)) {
            ROS_WARN("No value for minimal extension range specified. Looking for %s. Default is 1.0m.",
                    (ns + "/nbvpe/tree/min_extension_range").c_str());
        }
        params_.exact_root_ = true;
        if (!ros::param::get(ns + "/nbvpe/tree/exact_root", params_.exact_root_)) {
            ROS_WARN("No option for exact root selection specified. Looking for %s. Default is true.",
                    (ns + "/nbvpe/tree/exact_root").c_str());
        }
        params_.initIterations_ = 15;
        if (!ros::param::get(ns + "/nbvpe/tree/initial_iterations", params_.initIterations_)) {
            ROS_WARN("No number of initial tree iterations specified. Looking for %s. Default is 15.",
                    (ns + "/nbvpe/tree/initial_iterations").c_str());
        }
        params_.cuttoffIterations_ = 200;
        if (!ros::param::get(ns + "/nbvpe/tree/cuttoff_iterations", params_.cuttoffIterations_)) {
            ROS_WARN("No cuttoff iterations value specified. Looking for %s. Default is 200.",
                    (ns + "/nbvpe/tree/cuttoff_iterations").c_str());
        }
        params_.numberSamples_ = 200;
        if (!ros::param::get(ns + "/nbvpe/tree/number_samples", params_.numberSamples_)) {
            ROS_WARN("No cuttoff iterations value specified. Looking for %s. Default is 200.",
                    (ns + "/nbvpe/tree/number_samples").c_str());
        }
        paramsRePlanner_.numberSamples = params_.numberSamples_;
        params_.boundingBox_.x() = 0.5;
        if (!ros::param::get(ns + "/system/bbx/x", params_.boundingBox_.x())) {
            ROS_WARN("No x size Bounding Box value specified. Looking for %s. Default is 0.5m.", (ns + "/system/bbx/x").c_str());
        }
        params_.boundingBox_.y() = 0.5;
        if (!ros::param::get(ns + "/system/bbx/y", params_.boundingBox_.y())) {
            ROS_WARN("No y size Bounding Box value specified. Looking for %s. Default is 0.5m.", (ns + "/system/bbx/y").c_str());
        }
        paramsRePlanner_.boundingBox = params_.boundingBox_;
        params_.mapWeightBox_.x() = 1.5;
        if (!ros::param::get(ns + "/system/mwbx/x", params_.mapWeightBox_.x())) {
            ROS_WARN("No x size Weight Box value specified. Looking for %s. Default is 1.5m.", (ns + "/system/mwbx/x").c_str());
        }
        params_.mapWeightBox_.y() = 1.5;
        if (!ros::param::get(ns + "/system/mwbx/y", params_.mapWeightBox_.y())) {
            ROS_WARN("No y size Weight Box value specified. Looking for %s. Default is 1.5m.", (ns + "/system/mwbx/y").c_str());
        }
        paramsRePlanner_.replanRange = 0.5;
        if (!ros::param::get(ns + "/nbvpe/replan/range", paramsRePlanner_.replanRange)) {
            ROS_WARN("No cuttoff iterations value specified. Looking for %s. Default is 0.5.",
                    (ns + "/nbvpe/replan/range").c_str());
        }
        paramsRePlanner_.sampleRange = 0.5;
        if (!ros::param::get(ns + "/nbvpe/replan/sample_range", paramsRePlanner_.sampleRange)) {
            ROS_WARN("No cuttoff iterations value specified. Looking for %s. Default is 0.5.",
                    (ns + "/nbvpe/replan/sample_range").c_str());
        }
        
        return ret;
    }

    void Planner::posCallback(const geometry_msgs::PoseWithCovarianceStamped& pose) {
        tree_->setStateFromPoseMsg(pose);
    }

    void Planner::odomCallback(const nav_msgs::Odometry& pose) {
        tree_->setStateFromOdometryMsg(pose);
    }

    void Planner::mapCallback(const nav_msgs::OccupancyGrid& msg) {
        geometry_msgs::PoseWithCovarianceStamped robPos = tree_->setStateFromMapMsg(msg);
        rePlanner_->setMap(msg);
        map_ = msg;
        resolution_ = map_.info.resolution;
        cd_.setMap(map_);
        mapReady_ = true;

        robPosPub_.publish(robPos);
    }

    bool Planner::plannerCallback(windplanner::rhep_srv::Request& req,
                                                          windplanner::rhep_srv::Response& res) {
        ros::Time computationTime = ros::Time::now();

        // Clear old tree
        tree_->clear();
        
        // Check that planner is ready to compute path.
        if (!ros::ok()) {
            ROS_INFO("Exploration finished. Not planning any further moves.");
            return false;
        }
        if (!initialized_) {
            ROS_WARN("Planner not set up: Wait For TF!");
            return false;
        }
        if (!mapReady_) {
            ROS_WARN("Planner not set up: Wait For Map!");
            return false;
        }
        
        res.path.clear();

        params_.visualizeTrees_->deleteAllMarkers();
        params_.visualizeTreePoints_->deleteAllMarkers();

        // Reinitialize tree
        tree_->initialize();
        
        // Iterate the tree construction method.
        uint32_t loopCount = 0;
        while ((!tree_->gainFound() || tree_->getCounter() < params_.initIterations_) && ros::ok()) {
            if (tree_->getCounter() > params_.cuttoffIterations_) {
                ROS_INFO("No gain found, shutting down");
                ros::shutdown();
                return false;
            }
            if (loopCount > 1000 * (tree_->getCounter() + 1)) {
                ROS_INFO_THROTTLE(1, "Exceeding maximum failed iterations, return to previous point!");
                res.path = tree_->getPathBackToPrevious(req.header.frame_id);
                return false;
            }
            tree_->iterate();
            loopCount++;
        }
        
        // Extract the best edge.
        geometry_msgs::PoseArray computedPath;
        computedPath.header = req.header;
        computedPath.poses = tree_->getBestEdge(req.header.frame_id);
        computedPath.poses.pop_back();
        res.path = computedPath.poses;
        pathPub_.publish(computedPath);

        tree_->memorizeBestBranch();

        params_.visualizeTrees_->trigger();
        params_.visualizeTreePoints_->trigger();

        // Compute the time needed to compute the path.
        ROS_INFO("Path computation lasted %2.3fs", (ros::Time::now() - computationTime).toSec());
        return true;
    }

    bool Planner::rePlannerCallback(windplanner::rhep_resrv::Request& req,
                                                          windplanner::rhep_resrv::Response& res) {
        weightMapPub_.publish(tree_->getWeightMap());

        rePlanner_->setPaths(req.path);

        if (req.init.data) {
            res.path = rePlanner_->initialize();
        }
        else {
            res.path = rePlanner_->iterate();
        }

        // res.path = req.path;

        return true;
    }
}