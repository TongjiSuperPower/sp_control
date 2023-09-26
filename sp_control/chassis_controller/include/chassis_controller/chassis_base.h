#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <effort_controllers/joint_velocity_controller.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sp_common/ChassisCmd.h>

#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Odometry.h>

#include <angles/angles.h>
#include <string>

#include <sp_common/filters/filters.h>
#include <sp_common/base_utilities.h>
namespace chassis_controller
{
    struct Command
    {
        geometry_msgs::Twist cmd_vel_;
        sp_common::ChassisCmd cmd_chassis_;
        ros::Time stamp_;
    };

    class ChassisBase : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface>
    {
    public:
        ChassisBase() = default;
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
        virtual void moveJoint(const ros::Time &time, const ros::Duration &period) = 0;

        virtual geometry_msgs::Twist forwardKinematics() = 0;

        void follow(const ros::Time &time, const ros::Duration &period);

        void nofollow();

        void gyro();

        void recovery();

        /** @brief Write current command from  geometry_msgs::Twist.
         *
         *  @param msg This expresses velocity in free space broken into its linear and angular parts.
         */
        void tfVelToBase(const std::string& from);

        void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);

        void cmdChassisCallback(const sp_common::ChassisCmd::ConstPtr &cmd);

        hardware_interface::EffortJointInterface *effort_joint_interface_{};
        std::vector<hardware_interface::JointHandle> joint_handles_{};

        ros::Time last_publish_time_;
        geometry_msgs::TransformStamped odom2base_{};
        geometry_msgs::Vector3 vel_cmd_{}; // x, y
        control_toolbox::Pid pid_follow_;

        double wheel_base_{}, wheel_track_{}, wheel_radius_{}, publish_rate_{}, twist_angular_{},
            timeout_{}, effort_coeff_{}, velocity_coeff_{}, power_offset_{};
        ros::Subscriber cmd_chassis_sub_;
        ros::Subscriber cmd_vel_sub_;

        enum
        {
            FOLLOW,
            NOFOLLOW,
            GYRO
        };

        int state_ = NOFOLLOW;
        bool state_changed_ = false;

        Command cmd_struct_;
        realtime_tools::RealtimeBuffer<Command> cmd_rt_buffer_;
        std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odom_pub_;
        std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::TFMessage>> tf_odom_pub_;
        void setOdomPubFields(ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh);
        void updateOdom(const ros::Time &time, const ros::Duration &period);

        std::string odom_frame_id_ = "odom";
        std::string base_frame_id_ = "base_link";

        sp_common::RampFilter<double> *ramp_x_{}, *ramp_y_{}, *ramp_z_{};
    };

}
