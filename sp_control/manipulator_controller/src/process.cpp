#include "manipulator_controller/manipulator_controller.h"
#include <pluginlib/class_list_macros.hpp>
namespace manipulator_controller
{
    void ManipulatorController::orePlaceProcess()
    {    
        if (!planed_)
        {
            joint_pos_cmd_[2] = 0.2;
            joint_pos_cmd_[3] = 0.0;
            joint_pos_cmd_[6] = -1.57;
            generateSplineCoeff(2);
            generateSplineCoeff(3);
            generateSplineCoeff(6);
            begin_time_ = ros::Time::now();
            planed_ = true;
        } 

        if (planed_)
        {
            ros::Duration duration;
            now_time_ = ros::Time::now();
            duration = now_time_ - begin_time_;
            double sec = duration.toSec();
            bool reached = true;

            for (int i = 0; i < 7; i++)
            { 
               if (i == 2 || i == 3 || i == 6)
               { 
                    if (abs(joint_pos_cmd_[i] - joint_cmd_[i]) > 0.002)
                        joint_cmd_[i] = coeff_(i, 0) + coeff_(i, 1)*sec + coeff_(i, 2)*pow(sec, 2) + coeff_(i, 3)*pow(sec, 3);
                    else
                        joint_cmd_[i] = joint_pos_cmd_[i];
                    if (abs(joint_pos_cmd_[i] - joint_pos_[i]) > position_threshold_[i])
                        reached = false;
                }
            } 

            if (reached)
            {
                planed_ = false;
            }  
        }

 

            
    }

}
