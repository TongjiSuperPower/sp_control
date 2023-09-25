#include "sp_engineer/operator.h"

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

            ros::NodeHandle controller_nh;

            double x_coeff_, y_coeff_, z_rc_coeff_, z_mk_coeff_;
            double x_accel_set_, y_accel_set_, z_accel_set_;
            double yaw_coeff_, pitch_mk_coeff_, pitch_rc_coeff_;
            double yaw_left_limit_, yaw_right_limit_, pitch_low_limit_, pitch_high_limit_;
            
            enum
            {
                FOLLOW,
                NOFOLLOW,
                GYRO
            };

            
    };
}