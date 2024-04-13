#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <effort_controllers/joint_velocity_controller.h>
#include <effort_controllers/joint_position_controller.h>
#include <angles/angles.h>
#include <string>

#include <sp_common/filters/filters.h>
#include <sp_common/base_utilities.h>

#include <sp_common/DbusData.h>

namespace adjustment_controller
{

    class AdjustmentController: public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface>
    {
    public:
        AdjustmentController() = default;
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
        bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)  override;
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
        void starting(const ros::Time &time) override;
        void stopping(const ros::Time &time) override;


    private:

        void dbus_callback(const sp_common::DbusData::ConstPtr &dbus_data);

        double cmd_{};

        ros::NodeHandle nh;

        ros::Publisher pos_pub_;

        ros::Publisher vel_pub_;

        ros::Subscriber cmd_sub_;

        ros::Subscriber dbus_sub_;

        hardware_interface::EffortJointInterface *effort_joint_interface_{};

        effort_controllers::JointPositionController ctrl_pos_;
        
        effort_controllers::JointVelocityController ctrl_vel_;

        std_msgs::Float64 pos_{}, vel_{};

        double cmd_vel_{};

        sp_common::DbusData dbus_data_{};

        sp_common::RampFilter<double> *ramp_x_{};

        
    };

}
