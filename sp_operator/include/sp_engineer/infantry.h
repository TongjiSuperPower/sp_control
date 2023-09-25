#include "sp_engineer/operator.h"

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

            ros::NodeHandle controller_nh;

            double x_coeff_, y_coeff_, z_rc_coeff_, z_mk_coeff_;
            double x_accel_set_, y_accel_set_, z_accel_set_;
            
            enum
            {
                FOLLOW,
                NOFOLLOW,
                GYRO
            };

            
    };
}