#include <keyboard_control/keyboard_control.h>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "keyboard_control");
    KeyboradControl keyboard_control_;

    signal(SIGINT,quit);
    ros::Rate loop(5);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    while(ros::ok())
    {
        keyboard_control_.keyLoop();
        loop.sleep();
    }

   
    quit(0);
    return(0);
}


