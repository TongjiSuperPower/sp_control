#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#define PLANNING_GROUP_MANIPULATOR "engineer_manipulator" 
#define PLANNING_GROUP_GRIPPER  "gripper" 

namespace manipulator_control
{
    enum execution_mode{POSE, STATE};




    class Manipulator
    {
        public:
            Manipulator(moveit::planning_interface::MoveGroupInterface &move_group_interface,
                        moveit::planning_interface::MoveGroupInterface &grip_group_interface);

            bool init();

            void execute();

            void read();

            void write(const geometry_msgs::PoseStamped &target_pose_);

            bool get_executed();

            void set_executed(bool exe);
            geometry_msgs::PoseStamped target_pose;


        private:
            moveit::planning_interface::MoveGroupInterface &move_group_interface;
            moveit::planning_interface::MoveGroupInterface &grip_group_interface;
            const moveit::core::JointModelGroup* joint_model_group;
            const moveit::core::JointModelGroup* grip_model_group;
            geometry_msgs::PoseStamped current_pose;
            std::vector<double> current_state, target_state;
            execution_mode EXECUTION_MODE;
            bool executed;
            
    };

}