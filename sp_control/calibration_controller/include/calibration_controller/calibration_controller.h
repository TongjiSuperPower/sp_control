#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <sp_common/hardware_interface/gpio_interface.h>
#include <gpio_controller/gpio_controller.h>

#include <std_msgs/Bool.h>

#include <sp_common/filters/filters.h>
#include <sp_common/base_utilities.h>
#include "sp_common/CalibrationState.h"
namespace calibration_controller
{

    class CalibrationController: public controller_interface::MultiInterfaceController<sp_control::GpioCommandInterface>
    {
    public:
        CalibrationController() = default;
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
        void starting(const ros::Time &time) override;
        void stopping(const ros::Time &time) override;


    private:

        ros::Publisher joint_z_msg_pub_, joint_x_msg_pub_, joint_y_msg_pub_, joint_pitch_msg_pub_;

        sp_control::GpioCommandInterface *gpio_command_interface_{};

        gpio_controller::GpioController ctrl_joint_z_, ctrl_joint_x_, ctrl_joint_y_, ctrl_joint_pitch_;
      
    };

}
