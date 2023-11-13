#include "sp_engineer/operator.h"
#include "sp_common/ShooterCmd.h"

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

            void shooter_set();

            ros::NodeHandle controller_nh;

            double x_coeff_, y_coeff_, z_rc_coeff_, z_mk_coeff_;
            double x_accel_set_, y_accel_set_, z_accel_set_;

            sp_common::ShooterCmd shooter_cmd_;

            
    };
}