#ifndef RANDOM_DOUBLE_GENERATOR_HPP_  // NOLINT
#define RANDOM_DOUBLE_GENERATOR_HPP_

#include <ros/ros.h>
#include <Eigen/Dense>
#include <random>


namespace windPlanner {

    class RandomDoubleGenerator {
        private:
            double origin_x_{0.0};
            double origin_y_{0.0};
            uint32_t size_x_{0};
            uint32_t size_y_{0};
            double resolution_{0.0};

            std::default_random_engine gen_;

        public:
            RandomDoubleGenerator() = default;

            void setParams(double origin_x, double origin_y, uint32_t size_x, uint32_t size_y, double resolution);

            Eigen::Vector2d generate();
            Eigen::Vector2d generate(std::vector<double> weight);
    };
    
}  // namespace windPlanner

#endif  // RANDOM_DOUBLE_GENERATOR_HPP_  // NOLINT
