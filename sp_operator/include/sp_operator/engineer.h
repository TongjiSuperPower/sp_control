#include "sp_operator/operator.h"
#include "sp_common/ManipulatorCmd.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <control_msgs/JointJog.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>

namespace sp_operator
{
    class Engineer : Operator
    {
        public:
            Engineer() = default;

            bool init();

            void run();

        private:

            void chassis_set();

            void gimbal_set();

            void manipulator_set();

            void velocity_callback(const geometry_msgs::Twist::ConstPtr &vel);

            ros::NodeHandle controller_nh;

            ros::Publisher manipulator_cmd_pub_;

            ros::Publisher ore_cmd_pub_;

            ros::Publisher pump_cmd_pub_;

            ros::Publisher rod_cmd_pub_;

            ros::Publisher twist_cmd_pub_;

            ros::Publisher joint_cmd_pub_;

            ros::Publisher gimbal_calibration_pub_;

            ros::Publisher chassis_deflected_pub_;

            sp_common::ManipulatorCmd manipulator_cmd_{};

            geometry_msgs::Twist twist_cmd_{};

            //cmds for publish

            std_msgs::Float64MultiArray joint_vel_cmd_{};
            std_msgs::Int8 ore_cmd_;
            std_msgs::Bool pump_cmd_;
            std_msgs::Bool rod_cmd_;
            std_msgs::Bool deflection_cmd_;

            int deflection_change_count_ {};
            int pump_change_count_ {};
            int rod_change_count_ {};
            int calibration_change_count_ {};

            //control_msgs::JointJog joint_cmd_{};
            double timeout_;

            double x_coeff_, y_coeff_, z_rc_coeff_, z_mk_coeff_;
            double x_accel_set_, y_accel_set_, z_accel_set_;
            double yaw_coeff_, pitch_mk_coeff_, pitch_rc_coeff_;
            double yaw_left_limit_, yaw_right_limit_, pitch_low_limit_, pitch_high_limit_;
            
            enum
            {
                MAUL,
                AUTO,
                JOINT
            };

            enum
            {
                DESTINATION,
                PROCESS,  
            };

            enum
            {  
                NONE,
                HOME,
                GROUND,
                SLIVER,
                GOLD,
                EXCHANGE,
                VISION
            };

            enum
            {  
                STOP,
                TAKE_SLIVER,
                TAKE_GOLD,
            };

            ros::Subscriber velocity_sub_;


            geometry_msgs::Twist velocity_cmd_{};

            std_msgs::Bool gimbal_cali_cmd_{};

            
    };
}