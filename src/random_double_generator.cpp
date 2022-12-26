#include "windplanner/random_double_generator.hpp"

namespace windPlanner {

    void RandomDoubleGenerator::setParams(double origin_x, double origin_y, uint32_t size_x, uint32_t size_y, double resolution) {
        origin_x_ = origin_x;
        origin_y_ = origin_y;
        size_x_ = size_x;
        size_y_ = size_y;
        resolution_ = resolution;
    }
    
    Eigen::Vector2d RandomDoubleGenerator::generate() {
        std::uniform_int_distribution<int> distribution(0, static_cast<int>(size_x_ * size_y_));
        
        double x, y;

        auto distributeIndex = distribution(gen_);
        x = origin_x_ + distributeIndex % size_x_ * resolution_;
        y = origin_y_ + distributeIndex / size_x_ * resolution_;
        return Eigen::Vector2d(x, y);
    }

    Eigen::Vector2d RandomDoubleGenerator::generate(std::vector<double> weight) {
        std::discrete_distribution<int32_t> distribution(weight.begin(), weight.end());
        
        double x, y;

        auto distributeIndex = distribution(gen_);
        x = origin_x_ + distributeIndex % size_x_ * resolution_;
        y = origin_y_ + distributeIndex / size_x_ * resolution_;
        return Eigen::Vector2d(x, y);
        
    }

}  // namespace windPlanner
