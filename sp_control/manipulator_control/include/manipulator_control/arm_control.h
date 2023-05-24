#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <sp_common/GpioData.h>
#define PLANNING_GROUP_MANIPULATOR "engineer_manipulator"
#define PLANNING_GROUP_GRIPPER "gripper"
#include <moveit_visual_tools/moveit_visual_tools.h>

#define joint_eff 0.03

namespace manipulator_control
{

    enum execution_mode
    {
        POSE,
        STATE
    };

    class Manipulator
    {
    public:
        Manipulator(moveit::planning_interface::MoveGroupInterface &move_group_interface,
                    moveit::planning_interface::MoveGroupInterface &grip_group_interface);

        bool init();

        void move_execute();

        void move_execute(moveit_msgs::RobotTrajectory &trajectory);

        void move_execute(std::string goal);

        void joint_execute();

        void grip_execute();

        void read();

        void write(const geometry_msgs::Pose &target_pose_);

        void write(const std::vector<double> &target_state_);

        void singlewrite(double target_state_, int num);

        void singleaddwrite(double delta_theta, int num);

        void CartesianPath(std::vector<geometry_msgs::Pose> waypoints);

        bool get_executed();

        void set_executed(bool exe);

        void stretch(const std::vector<double> &distance);

        void goal(const std::string &name);

        //void suck(const bool &suck_it);

       // void sucker_callback(const sp_common::GpioData::ConstPtr &gpio_data_);

    private:
        moveit::planning_interface::MoveGroupInterface &move_group_interface;
        moveit::planning_interface::MoveGroupInterface &grip_group_interface;
        const moveit::core::JointModelGroup *joint_model_group;
        const moveit::core::JointModelGroup *grip_model_group;
        geometry_msgs::Pose current_pose, target_pose;
        std::vector<double> current_state, target_state, current_distance, target_distance;
        execution_mode EXECUTION_MODE;
        bool executed;
        ros::NodeHandle nh_;
        ros::Publisher pose_publisher_;
        //ros::Publisher sucker_pub_;
       // ros::Subscriber sucker_sub_;
       // sp_common::GpioData gpio_data;
        //int gpio_size;
    };

}