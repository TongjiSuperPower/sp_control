#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <position_controllers/joint_position_controller.h>
#include <effort_controllers/joint_position_controller.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Odometry.h>

#include <sp_common/filters/filters.h>
namespace gimbal_controller
{
    struct Command
    {
        geometry_msgs::Vector3 cmd_pos_;
        ros::Time stamp_;
    };

    class GimbalController : public controller_interface::MultiInterfaceController<hardware_interface::PositionJointInterface, hardware_interface::EffortJointInterface>
    {
    public:
        GimbalController() = default;
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

    protected:
        /** @brief Write current command from  geometry_msgs::Twist.
         *
         * @param msg This expresses velocity in free space broken into its linear and angular parts.
         */
        void cmdPosCallback(const geometry_msgs::Vector3::ConstPtr &msg);

        double publish_rate_{}, timeout_{};

        hardware_interface::PositionJointInterface *position_joint_interface_{};
        hardware_interface::EffortJointInterface *effort_joint_interface_{};
        std::vector<hardware_interface::JointHandle> joint_handles_{};

        ros::Time last_publish_time_;
        geometry_msgs::Vector3 pos_cmd_{}; // x, y

        ros::Subscriber cmd_gimbal_sub_;
        ros::Subscriber cmd_pos_sub_;

        Command cmd_struct_;
        realtime_tools::RealtimeBuffer<Command> cmd_rt_buffer_;

    private:
        void moveJoint(const ros::Time &time, const ros::Duration &period);

        position_controllers::JointPositionController ctrl_yaw_, ctrl_pitch_;
        effort_controllers::JointPositionController ctrl_height_;
    };

}
