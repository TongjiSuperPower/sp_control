#include "manipulator_control/arm_control.h"

namespace manipulator_control
{

    Manipulator::Manipulator(moveit::planning_interface::MoveGroupInterface &move,
                             moveit::planning_interface::MoveGroupInterface &grip)
        : move_group_interface(move), grip_group_interface(grip)
    {
    }

    bool Manipulator::init()
    {
        joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP_MANIPULATOR);
        grip_model_group = grip_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP_GRIPPER);
        executed = true;
        move_group_interface.setPoseReferenceFrame("base_link");
        move_group_interface.setGoalPositionTolerance(0.015);
        move_group_interface.setGoalOrientationTolerance(0.015);
        move_group_interface.setGoalJointTolerance(0.015);
        move_group_interface.setEndEffectorLink("link6");
        grip_group_interface.setPoseReferenceFrame("base_link");
        grip_group_interface.setGoalPositionTolerance(0.01);
        grip_group_interface.setGoalOrientationTolerance(0.01);
        //sucker_pub_ = nh_.advertise<sp_common::GpioData>("/controllers/gpio_controller/command", 1000);
        //sucker_sub_ = nh_.subscribe<sp_common::GpioData>("/controllers/gpio_controller/state", 10, boost::bind(&Manipulator::sucker_callback, this, _1));
        pose_publisher_ = nh_.advertise<geometry_msgs::Pose>("calibrate", 1000);
        EXECUTION_MODE = POSE;
       // gpio_size = 4;
        return true;
    }

    void Manipulator::read()
    {
        current_pose = move_group_interface.getCurrentPose("vacuum_gripper").pose;
        current_state = move_group_interface.getCurrentJointValues();
        current_distance = grip_group_interface.getCurrentJointValues();
        ROS_INFO_STREAM(current_pose<<"   calibrate   ");
        ROS_INFO_STREAM("[ joint1: " << current_state[0] << " joint2: " << current_state[1] << " joint3: " << current_state[2] << " joint4: " << current_state[3] << " joint5: " << current_state[4] << " joint6: " << current_state[5] << " ]" << std::endl);
        ROS_INFO_STREAM("joint7: " << current_distance[0]);
        pose_publisher_.publish(current_pose);
    }

    void Manipulator::write(const geometry_msgs::Pose &target_pose_)
    {
        target_pose = target_pose_;
        EXECUTION_MODE = POSE;
        if (executed == true)
            executed = false;
        move_execute();
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
        move_group_interface.setJointValueTarget(target_state);   
        joint_execute();  
        
    }

    void Manipulator::singlewrite(double target_state_, int num)
    {
        read();
        target_state = current_state;
        target_state[num] = target_state_;
        EXECUTION_MODE = STATE;
        if (executed == true)
            executed = false;
        move_group_interface.setJointValueTarget(target_state);
        joint_execute();    
    }

    void Manipulator::singleaddwrite(double delta_theta, int num)
    {
        read();
        target_state = current_state;
        target_state[num - 1] += delta_theta;
        EXECUTION_MODE = STATE;
        if (executed == true)
            executed = false;
        move_group_interface.setJointValueTarget(target_state);    
        joint_execute(); 
    }

    void Manipulator::CartesianPath(std::vector<geometry_msgs::Pose> waypoints)
    {
        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;                                                                            // 跳跃阈值
        const double eef_step = 0.01;                                                                                 // 终端步进值
        double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory); // 规划路径 ，fraction返回1代表规划成功
        ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
        if (fraction == 1)
        {
            ROS_INFO_STREAM("SUCCEED TO CREATE A CARTESIAN PATH!");
        }
        move_execute(trajectory);
    }

    void Manipulator::move_execute()
    {
        moveit_msgs::RobotTrajectory best_trajectory;
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        int shortest = 1000;
        ROS_INFO_STREAM("Begin to plan");
        double j4_delta, j5_delta, j6_delta;
        j4_delta = j5_delta = j6_delta = 0.0;
        while (1)
        {
            move_group_interface.setApproximateJointValueTarget(target_pose, "link6");
            bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            int tra_size = 0;
            if (success)
            {
                tra_size = my_plan.trajectory_.joint_trajectory.points.size();
                j4_delta = abs(my_plan.trajectory_.joint_trajectory.points[0].positions[3] - my_plan.trajectory_.joint_trajectory.points[tra_size - 1].positions[3]);
                j5_delta = abs(my_plan.trajectory_.joint_trajectory.points[0].positions[4] - my_plan.trajectory_.joint_trajectory.points[tra_size - 1].positions[4]);
                j6_delta = abs(my_plan.trajectory_.joint_trajectory.points[0].positions[5] - my_plan.trajectory_.joint_trajectory.points[tra_size - 1].positions[5]);
                if (j4_delta > 2.60 || j5_delta > 2.60 || j6_delta > 2.60)
                    continue;
                else
                {
                    best_trajectory = my_plan.trajectory_;
                    break;
                }              
            }  
            ROS_INFO_STREAM(my_plan.trajectory_.joint_trajectory.points.size());          
        }     
        ROS_INFO_STREAM(my_plan.trajectory_.joint_trajectory);
        move_group_interface.execute(best_trajectory);
    }

    void Manipulator::move_execute(moveit_msgs::RobotTrajectory &trajectory)
    {
        // moveit_visual_tools::MoveItVisualTools visual_tools;
        // visual_tools.publishTrajectoryLine(trajectory, joint_model_group);
        // visual_tools.trigger();
        move_group_interface.setStartStateToCurrentState();
        move_group_interface.execute(trajectory);
    }

    void Manipulator::move_execute(std::string name)
    {
        moveit_msgs::RobotTrajectory best_trajectory;
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        int shortest = 1000;
        ROS_INFO_STREAM("Begin to plan");
        for (int i = 0; i < 10; i++)
        {
            move_group_interface.setNamedTarget(name);
            bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if (success)
            {
                if (shortest > my_plan.trajectory_.joint_trajectory.points.size())
                {
                    shortest = my_plan.trajectory_.joint_trajectory.points.size();
                    best_trajectory = my_plan.trajectory_;
                }
            }  
           
        }
        
        ROS_INFO_STREAM("tutorial Visualizing plan 1 (pose goal)");
        move_group_interface.execute(best_trajectory);
    }

    void Manipulator::joint_execute()
    {
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_STREAM("tutorial Visualizing plan 1 (pose goal)" << success ? "" : "FAILED");
        move_group_interface.execute(my_plan);
    }

    

    void Manipulator::grip_execute()
    {

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (grip_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (grip distance goal) %s", success ? "" : "FAILED");
        grip_group_interface.execute(my_plan);
    }

    bool Manipulator::get_executed()
    {
        return executed;
    }

    void Manipulator::set_executed(bool exe)
    {
        executed = exe;
    }

    void Manipulator::stretch(const std::vector<double> &distance)
    {
        target_distance = distance;
        grip_group_interface.setJointValueTarget(target_distance);
    }

    void Manipulator::goal(const std::string &name)
    {
        move_execute(name);
    }

    /*void Manipulator::suck(const bool &suck_it)
    {
        for (int i = 0; i < gpio_size; i++ )
        {
            
            if (gpio_data.gpio_name[i] == "sucker")
            {
                if (suck_it)
                    gpio_data.gpio_state[i] = true;
                else
                    gpio_data.gpio_state[i] = false;

            }
           

        }
        
        sucker_pub_.publish(gpio_data);
    }

    void Manipulator::sucker_callback(const sp_common::GpioData::ConstPtr &gpio_data_)
    {
        gpio_data = *gpio_data_;
        gpio_size = gpio_data_->gpio_name.size();
    }*/

}
