#include "manipulator_control/arm_control.h"
#include "manipulator_control/scene_generate.h"
#include <std_msgs/Float64MultiArray.h>
#include "sp_common/SingleJointWrite.h"
#include "sp_common/DbusData.h"

void pose_callback(const geometry_msgs::Pose::ConstPtr &pose_, manipulator_control::Manipulator *manipulator)
{
    manipulator->write(*pose_);
}

void state_callback(const std_msgs::Float64MultiArray::ConstPtr &state_, manipulator_control::Manipulator *manipulator)
{
    std::vector<double> state;
    state = state_->data;
    manipulator->write(state);
}

void single_state_callback(const sp_common::SingleJointWrite::ConstPtr &state_, manipulator_control::Manipulator *manipulator)
{
    double state;
    int num;
    state = state_->state;
    num = state_->num;
    manipulator->singlewrite(state, num);
}

// void remote_control_callback(const sp_common::DbusData::ConstPtr &data_, )

// void remote_control_callback(const sp_common::RCData::ConstPtr& RCData_, const sp_control::RCData & RCData)
//{
//
// }
void auto_take_silver_ore(manipulator_control::Manipulator *manipulator_, geometry_msgs::Pose target_pose)
{
    geometry_msgs::Pose pose1;
    geometry_msgs::Pose pose2;
    std::vector<double> stretch1;
    std::vector<double> stretch2;
    pose1.position.x = 0.00;
    pose1.position.y = 0.48;
    pose1.position.z = 0.280;
    pose1.orientation.w = 0.00;
    pose1.orientation.x = -1.00;
    pose1.orientation.y = 0.00;
    pose1.orientation.z = 0.00;
    pose2.position.x = 0.044;
    pose2.position.y = -0.100;
    pose2.position.z = 0.433;
    pose2.orientation.w = 0.00;
    pose2.orientation.x = 0.00;
    pose2.orientation.y = 1.00;
    pose2.orientation.z = 0.00;
    stretch1.push_back(-0.075);
    stretch2.push_back(0.00);
    manipulator_->read();
    manipulator_->goal("home");
    manipulator_->move_execute();
    sleep(0.3);
    manipulator_->read();
    manipulator_->write(target_pose);
    manipulator_->move_execute();
    sleep(0.3);
    manipulator_->read();
    manipulator_->stretch(stretch1);
    manipulator_->grip_execute();
    sleep(0.3);
    manipulator_->read();
    manipulator_->stretch(stretch2);
    manipulator_->grip_execute();
    sleep(0.3);
    manipulator_->read();
    manipulator_->write(pose2);
    manipulator_->move_execute();
    sleep(0.3);
    manipulator_->read();
    manipulator_->stretch(stretch1);
    manipulator_->grip_execute();
    sleep(0.3);
    manipulator_->read();
    manipulator_->stretch(stretch2);
    manipulator_->grip_execute();
    sleep(0.3);
    manipulator_->read();
    manipulator_->goal("home");
    manipulator_->move_execute();
}

int main(int argc, char **argv)
{
ros::init(argc, argv, "trajectory_control", ros::init_options::AnonymousName);
ros::AsyncSpinner spinner(3);
moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP_MANIPULATOR);
moveit::planning_interface::MoveGroupInterface grip_group_interface(PLANNING_GROUP_GRIPPER);
manipulator_control::Manipulator manipulator_(move_group_interface, grip_group_interface);
geometry_msgs::Pose pose1;
// ros::Rate rate(10);
spinner.start();
ros::init(argc,argv,"calibrate");
ros::NodeHandle nh;
ros::Publisher pub = nh.advertise<geometry_msgs::Pose>("pose",10);
rate.sleep();
while(ros::ok())
{
    pose1 = move_group_interface.getCurrentPose().pose;
    //逻辑(一秒10次)
    ros::Rate r(1);
    pub.publish(pose1)
return 0;
}
}

