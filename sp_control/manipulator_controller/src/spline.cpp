//
// Created by CherryBlossomNight on 2024/2/18
//
#include "manipulator_controller/spline.h"
namespace manipulator_controller
{
    void Spline::init(double start, double end, double vel)
    {
        start_point_ = start;
        end_point_ = end;
        vel_ = vel;
        rough_time_ = abs((end_point_ - start_point_) / vel_);
        // ROS_INFO_STREAM(start);
        // ROS_INFO_STREAM(end);
        // ROS_INFO_STREAM(vels_);
        //ROS_INFO_STREAM("TIME:" << rough_time_[0] <<" "<< rough_time_[1] <<" "<<rough_time_[2] <<" "<< rough_time_[3]<<" " <<rough_time_[4]<<" " << rough_time_[5] <<" "<<rough_time_[6]);

    }

    void Spline::computeCoeff(Eigen::Vector4d &coeff)
    {

        coeff(0, 0) = start_point_;
        coeff(1, 0) = 0.0;
        coeff(2, 0) = 3 * (end_point_ - start_point_) / (pow(rough_time_, 2));
        coeff(3, 0) = 2 * (start_point_ - end_point_) / (pow(rough_time_, 3));

    }

}// namespace manipulator_controller