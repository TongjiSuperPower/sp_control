#pragma once
#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
namespace manipulator_controller
{
    class Spline
    {
        public:
            void init(double start, double end, double vel);

            void computeCoeff(Eigen::Vector4d &coeff);

        private:

            double start_point_{};
            double end_point_{};
            double vel_{};
            int num_;
            double rough_time_{};


    };
}