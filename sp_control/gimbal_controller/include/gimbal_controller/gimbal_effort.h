#pragma once

#include <gimbal_controller/gimbal_base.h>
#include <control_toolbox/pid.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_listener.h>
#include <angles/angles.h>
namespace gimbal_controller
{
    
    class GimbalEffort : public GimbalBase
    {
    public:
        GimbalEffort() = default;
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

        hardware_interface::EffortJointInterface *effort_joint_interface_{};

    private:
        void moveJoint(const ros::Time &time, const ros::Duration &period);

        void cmdIMUCallback(const sensor_msgs::Imu::ConstPtr &msg);

        void initCmd(const ros::Time &time, const ros::Duration &period);

        void pubAngle();

        void getPosition(const ros::Time &time, const ros::Duration &period);

        void getAngle();

        double posLimit(double pos);

        void pubYawAngle();

        effort_controllers::JointPositionController ctrl_yaw_, ctrl_pitch_;

        ros::Publisher angle_pub_;

        ros::Subscriber imu_sub_;
        
        control_toolbox::Pid yaw_pid_, pitch_pid_;

        geometry_msgs::Quaternion quat_msg_{};

        double yaw_cmd_{}, pitch_cmd_{};


    };

}
