#include "sp_engineer/engineer.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sp_engineer");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::Rate loop(50);
    sp_operator::Engineer engineer;
    engineer.init();
    while(ros::ok())
    {
        engineer.run();
        loop.sleep();
    }
    return 0;
}


