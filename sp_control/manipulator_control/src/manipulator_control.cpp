#include "manipulator_control/arm_control.h"


void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_, manipulator_control::Manipulator* manipulator)
{
    geometry_msgs::PoseStamped pose;
    pose.header = pose_->header;
    pose.pose = pose_->pose;
    manipulator->write(pose);
  
    
    
}

//void state_callback(const std::vector<double>::ConstPtr& state)

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_control", ros::init_options::AnonymousName);
    ros::AsyncSpinner spinner(3);
   
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP_MANIPULATOR);
    moveit::planning_interface::MoveGroupInterface grip_group_interface(PLANNING_GROUP_GRIPPER);
    manipulator_control::Manipulator manipulator_(move_group_interface, grip_group_interface);
    ros::NodeHandle nh; 
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/moveit/pose_sub",10, boost::bind(&pose_callback, _1, &manipulator_));
    spinner.start(); 
    
    if (manipulator_.init())
    {
        ROS_WARN_STREAM("1");
        
        while(ros::ok())
        {
            
             
                   
            manipulator_.read();
            
           
       
            
           
            //ros::Subscriber state_sub = nh.subscribe<std_msgs::String>("/moveit/state_sub",10, state_callback)
            if (manipulator_.get_executed() == false)
            {
                ROS_WARN_STREAM(manipulator_.target_pose);
                manipulator_.execute();
                manipulator_.set_executed(true);
            }

            
         

            
            ros::spinOnce();
            sleep(1);
            
            
        }
    }    
    return 0;
}

