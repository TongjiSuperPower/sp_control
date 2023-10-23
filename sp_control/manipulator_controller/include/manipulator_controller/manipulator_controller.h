#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <effort_controllers/joint_position_controller.h>

#include <geometry_msgs/Quaternion.h>


#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Odometry.h>

#include <sp_common/filters/filters.h>
#include <sp_common/base_utilities.h>
#include <sp_common/ManipulatorCmd.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
namespace manipulator_controller
{
    struct Command
    {
        geometry_msgs::Quaternion cmd_quat_;
        sp_common::ManipulatorCmd cmd_manipulator_;
        ros::Time stamp_;
    };


    class ManipulatorController : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface>
    {
    public:
        ManipulatorController() = default;
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

        void cmdManipulatorCallback(const sp_common::ManipulatorCmd::ConstPtr &msg);

        void cmdQuatCallback(const geometry_msgs::Quaternion::ConstPtr &msg);

        void stopProcess();

        void readyProcess();

        double publish_rate_{}, timeout_{};

        hardware_interface::EffortJointInterface *effort_joint_interface_{};

        effort_controllers::JointPositionController ctrl_pitch_, ctrl_yaw_, ctrl_roll_;

        ros::Time last_publish_time_;
        Eigen::Quaterniond quat_cmd_{};
        Eigen::Vector3d euler_cmd_{}; 

        //ros::Subscriber cmd_gimbal_sub_;
        ros::Subscriber cmd_quat_sub_;

        enum
        {
            AUTO,
            MAUL,
            JOINT
        };

        enum
        {
            STOP,
            READY,
            MOVE,
            DONE
        };

        int mode_ = MAUL;
        bool mode_changed_ = false;

        int process_ = STOP;
        bool process_changed_ = false;



        Command cmd_struct_;
        realtime_tools::RealtimeBuffer<Command> cmd_rt_buffer_;

     
    };

}

