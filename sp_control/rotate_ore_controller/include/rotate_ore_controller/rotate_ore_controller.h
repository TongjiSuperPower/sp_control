#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <effort_controllers/joint_velocity_controller.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>


#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Odometry.h>

#include <sp_common/filters/filters.h>
#include <sp_common/base_utilities.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <std_msgs/Int8.h>。


namespace rotate_ore_controller
{
    struct Command
    {
        ros::Time stamp_;
        int ore_cmd_;
        float ore_vel_;
    };


    class RotateOreController : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface>
    {
    public:
        RotateOreController() = default;
        /** @brief Get and check params for covariances. Setup odometry realtime publisher + odom message constant fields.
         * init odom tf.
         *
         * @param robot_hw The robot hardware abstraction.
         * @param root_nh A NodeHandle in the root of the controller manager namespace. This is where the ROS interfaces are
         * setup (publishers, subscribers, services).
         * @param controller_nh A NodeHandle in the namespace of the controller. This is where the controller-specific
         * configuration resides.
         * @return True if initialization was successful and the controller
         * is ready to be started.
         */
        bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;
        /** @brief Receive real_time command from manual. Execute different action according to current mode. Set
         * necessary params of chassis. Execute power limit.
         *
         * Receive real_time command from manual and check whether it is normally, if can not receive command from manual
         * for a while, chassis's velocity will be set zero to avoid out of control. Execute different action according
         * to current mode such as RAW, FOLLOW, GYRO, TWIST.(Their specific usage will be explain in the next). UpdateOdom,Set
         * necessary params such as Acc and vel_tfed. Execute moving action and powerlimit.
         *
         * @param time The current time.
         * @param period The time passed since the last call to update.
         */
        void update(const ros::Time &time, const ros::Duration &period) override;
        void starting(const ros::Time &time) {}
        void stopping(const ros::Time &time) {}

    protected:
        void moveJoint(const ros::Time &time, const ros::Duration &period);

        void cmdOreCallback(const std_msgs::Int8::ConstPtr &msg);

        hardware_interface::EffortJointInterface *effort_joint_interface_{};

        // syn_vel_controller::SynVelController ctrl_ore_;
        effort_controllers::JointVelocityController ctrl_left_;
        effort_controllers::JointVelocityController ctrl_right_; 

        float vel_;
        int ore_cmd_;

        // Subscribers
        ros::Subscriber cmd_ore_sub_;

        Command cmd_struct_;
        realtime_tools::RealtimeBuffer<Command> cmd_rt_buffer_;

     
    };

}

