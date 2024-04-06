//
// Created by CherryBlossomNight on 2024/2/18
//
#include "manipulator_controller/spline.h"
namespace manipulator_controller
{
    void Spline::init(Eigen::Matrix<double, 7, 1> start, Eigen::Matrix<double, 7, 1> end, Eigen::Matrix<double, 7, 1> vel)
    {
        start_points_ = start;
        end_points_ = end;
        vels_ = vel;
        for (int i = 0; i < 7; i++)
            rough_time_.push_back(abs((end_points_[i] - start_points_[i]) / vels_[i]));

        ROS_INFO_STREAM("TIME:" << rough_time_[0] <<" "<< rough_time_[1] <<" "<<rough_time_[2] <<" "<< rough_time_[3]<<" " <<rough_time_[4]<<" " << rough_time_[5] <<" "<<rough_time_[6]);

    }

    void Spline::computeCoeff(Eigen::Matrix<double, 7, 4> &coeff)
    {
        for (int i = 0; i < 7; i++)
        {
            coeff(i, 0) = start_points_[i];
            coeff(i, 1) = 0.0;
            coeff(i, 2) = 3 * (end_points_[i] - start_points_[i]) / (pow(rough_time_[i], 2));
            coeff(i, 3) = 2 * (start_points_[i] - end_points_[i]) / (pow(rough_time_[i], 3));
        }
    }

}// namespace manipulator_controller