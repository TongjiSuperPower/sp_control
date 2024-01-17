#pragma once

#include <gimbal_controller/gimbal_base.h>
#include <std_msgs/Bool.h>
namespace gimbal_controller
{
    
    class GimbalEngineer : public GimbalBase
    {
    public:
        GimbalEngineer() = default;
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

        void update(const ros::Time &time, const ros::Duration &period);

    protected:
        /** @brief Write current command from  geometry_msgs::Twist.
         *
         * @param msg This expresses velocity in free space broken into its linear and angular parts.
         */

        hardware_interface::PositionJointInterface *position_joint_interface_{};
        hardware_interface::EffortJointInterface *effort_joint_interface_{};

    private:
        void moveJoint(const ros::Time &time, const ros::Duration &period);

        void getPosition(const ros::Time &time, const ros::Duration &period);

        void initCmd(const ros::Time &time, const ros::Duration &period);

        void GimbalCaliCallback(const std_msgs::Bool::ConstPtr &msg);

        effort_controllers::JointPositionController ctrl_yaw_;

        position_controllers::JointPositionController ctrl_pitch_;

        ros::Subscriber gimbal_cali_sub_;

        double yaw_cmd_{};

        bool init_pos_{}, init_cmd_{};

        bool initiated_{};
    };

}
