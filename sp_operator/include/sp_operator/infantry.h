#include "sp_operator/operator.h"
#include "sp_common/ShooterCmd.h"
#include <std_msgs/Bool.h>

namespace sp_operator
{
    class Infantry : public Operator
    {
        public:
            Infantry() = default;;

            bool init();

            void run();

        private:

            void chassis_set();

            void gimbal_set();

            void shooter_set();

            ros::NodeHandle controller_nh;

            double x_coeff_, y_coeff_, gyro_vel_;
            double x_accel_set_, y_accel_set_, z_accel_set_;
            double yaw_mk_coeff_, yaw_rc_coeff_, pitch_mk_coeff_, pitch_rc_coeff_;

            bool truned_;
            ros::Time trun_time_;

            ros::Publisher shooter_cmd_pub_;
            ros::Publisher cover_cmd_pub_;

            sp_common::ShooterCmd shooter_cmd_;

            std_msgs::Bool cmd_cover_;

         

            enum
            {
                FRIC_OFF,
                FRIC_ON
            };

            enum
            {
                SHOOT_STOP,
                SHOOT_READY,
                SHOOT_SINGLE,
                SHOOT_CONTINUOUS,
            };

            enum
            {
                CONTINUOUS_MODE,
                SINGLE_MODE
            };


            
    };
}