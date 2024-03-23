#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <effort_controllers/joint_position_controller.h>
#include <syn_controller/syn_controller.h>

#include <differential_controller/differential_controller.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float64MultiArray.h>


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
        geometry_msgs::Twist cmd_twist_;
        std_msgs::Float64MultiArray cmd_joint_;
        sp_common::ManipulatorCmd cmd_manipulator_;
        ros::Time stamp_;
    };

    struct Structure_coeff
    {
        double l1_;
        double l2_;
        double l3_;
        double l4_;
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

        void initPosition(const ros::Time &time, const ros::Duration &period);

        void maulMode();

        void autoMode();

        void jointMode();

        void quat2Euler();

        void calJointVel();

        void getPosition();

        void updateJacobian();
        
        void VelChangeConstraint();

        void finalPush();
        
        void jointPosConstraint();

        void cmdQuatCallback(const geometry_msgs::Quaternion::ConstPtr &msg);

        void cmdTwistCallback(const geometry_msgs::Twist::ConstPtr &msg);

        void cmdJointCallback(const std_msgs::Float64MultiArray::ConstPtr &msg);

        void cmdManipulatorCallback(const sp_common::ManipulatorCmd::ConstPtr &msg);

        void stopProcess();

        void readyProcess();

        bool simulate_{};

        double publish_rate_{}, timeout_{};

        hardware_interface::EffortJointInterface *effort_joint_interface_{};

        syn_controller::SynController ctrl_z_, ctrl_x_;

        effort_controllers::JointPositionController ctrl_y_;

        effort_controllers::JointPositionController ctrl_yaw_, ctrl_roll1_;

        effort_controllers::JointPositionController ctrl_pitch_, ctrl_roll2_;

        differential_controller::DifferentialController ctrl_diff_;

        ros::Time last_publish_time_;

        std::vector<double> vel_limit_{};
        std::vector<double> upper_pos_limit_{};
        std::vector<double> lower_pos_limit_{};

        Structure_coeff structure_coeff_{};

        Eigen::Quaterniond quat_cmd_{};

        Eigen::Matrix<double, 6, 1> twist_cmd_{};
        Eigen::Matrix<double, 6, 1> last_twist_cmd_{};
        Eigen::Matrix<double, 7, 1> joint_cmd_{}, joint_vel_cmd_{}, joint_pos_{}, joint_pos_cmd_{},vel_cmd_{};
        sp_common::ManipulatorCmd manipulator_cmd_{};

        Eigen::Vector4d cartesian_cmd_{};
        Eigen::Vector3d euler_cmd_{};
        Eigen::Vector4d xyz_cmd_{};
        Eigen::Vector3d rpy_cmd_{};

        Eigen::Matrix3d jacobian {};
        // Subscribers
        ros::Subscriber cmd_quat_sub_;
        ros::Subscriber cmd_twist_sub_;
        ros::Subscriber cmd_joint_sub_;
        ros::Subscriber cmd_manipulator_sub_;

        bool z_completed_{};
        bool x_completed_{};
        bool y_completed_{};
        bool rpy_completed_{};
        bool initiated_{};

        enum
        {
            MAUL,
            AUTO,
            JOINT
        };

        enum
        {
            STOP,
            READY,
            MOVE,
            DONE
        };

        enum
        {  
            HOME,
            GROUND,
            PLACE,
            LEFT90
        };

        int mode_ = MAUL;
        bool mode_changed_ = false;

        int process_ = STOP;
        bool process_changed_ = false;

        bool y_has_friction_{};
        double y_friction_{};

        bool last_final_push_{}, final_push_{};

        int orientation_ = HOME;
        ros::Time prev_time;
        double dt;



        Command cmd_struct_;
        realtime_tools::RealtimeBuffer<Command> cmd_rt_buffer_;

     
    };

}

