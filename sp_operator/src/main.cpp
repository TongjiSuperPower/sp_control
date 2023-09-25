#include "sp_engineer/engineer.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sp_engineer");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(2);
    spinner.start();
    sp_operator::Engineer engineer;
    engineer.init();
    while(ros::ok())
    {
        engineer.run();
    }
    return 0;
}


