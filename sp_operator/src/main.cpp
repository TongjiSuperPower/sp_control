#include "sp_operator/infantry.h"
// #include "sp_operator/engineer.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "sp_operator");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::Rate loop(50);
    // sp_operator::Engineer engineer;
    sp_operator::Infantry infantry;
    // engineer.init();
    infantry.init();
    while(ros::ok())
    {
        infantry.run();
        // engineer.run();
        loop.sleep();
    }
    return 0;
}


