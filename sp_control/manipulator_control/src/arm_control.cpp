#include "manipulator_control/arm_control.h"

namespace manipulator_control
{

    Manipulator::Manipulator(moveit::planning_interface::MoveGroupInterface &move,
                             moveit::planning_interface::MoveGroupInterface &grip)
                            :move_group_interface(move), grip_group_interface(grip)
    {
    }

    bool Manipulator::init()
    {
        joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP_MANIPULATOR);
        grip_model_group = grip_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP_GRIPPER);
        executed = true;
        move_group_interface.setPoseReferenceFrame("base_link");
        move_group_interface.setGoalPositionTolerance(0.05);
        move_group_interface.setGoalOrientationTolerance(0.05);
        grip_group_interface.setPoseReferenceFrame("base_link");
        grip_group_interface.setGoalPositionTolerance(0.005);
        grip_group_interface.setGoalOrientationTolerance(0.005);
        EXECUTION_MODE = POSE; 
        return true;

    }

    void Manipulator::read()
    {
        current_pose = move_group_interface.getCurrentPose().pose;
        current_state = move_group_interface.getCurrentJointValues();
    }

    void Manipulator::write(const geometry_msgs::Pose &target_pose_)
    {
        target_pose = target_pose_;  
        EXECUTION_MODE = POSE;
        if (executed == true)
            executed = false;    
    }

    void Manipulator::write(const std::vector<double> &target_state_)
    {
        if (target_state_.size() != 6)
        {
            ROS_WARN_STREAM("The number of the target_state is not 6!");
            return;
        }
        target_state = target_state_;
        EXECUTION_MODE = STATE;
        if (executed == true)
            executed = false; 
    }

    void Manipulator::singlewrite(double target_state_, int num)
    {
        read();
        target_state = current_state;
        target_state[num] = target_state_;
        EXECUTION_MODE = STATE;
        if (executed == true)
            executed = false; 
    }


    void Manipulator::execute()
    {
        //ROS_WARN_STREAM(target_pose);
        if (EXECUTION_MODE == POSE)
        {
            move_group_interface.setApproximateJointValueTarget(target_pose, "link6");
           
        }
        else 
        {
            move_group_interface.setJointValueTarget(target_state);
        }
        
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
        move_group_interface.execute(my_plan);
    }

    bool Manipulator::get_executed()
    {
        return executed;
    }

    void Manipulator::set_executed(bool exe)
    {
        executed = exe;
    }
}

