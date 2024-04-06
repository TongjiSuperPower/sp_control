#pragma once
#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
namespace manipulator_controller
{
    class Spline
    {
        public:
            void init(Eigen::Matrix<double, 7, 1> start, Eigen::Matrix<double, 7, 1> end, Eigen::Matrix<double, 7, 1> vel);

            void computeCoeff(Eigen::Matrix<double, 7, 4> &coeff);

        private:

            Eigen::Matrix<double, 7, 1> start_points_{};
            Eigen::Matrix<double, 7, 1> end_points_{};
            Eigen::Matrix<double, 7, 1> vels_{};
            int num_;
            std::vector<double> rough_time_;


    };
}