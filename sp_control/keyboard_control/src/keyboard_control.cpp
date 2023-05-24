#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <stdio.h>
#ifndef _WIN32
# include <termios.h>
# include <unistd.h>
#else
# include <windows.h>
#endif


class KeyboardReader
{
    public:
    KeyboardReader()
    #ifndef _WIN32
        : kfd(0)
    #endif
    {
        #ifndef _WIN32
        // get the console in raw mode
        tcgetattr(kfd, &cooked);
        struct termios raw;
        memcpy(&raw, &cooked, sizeof(struct termios));
        raw.c_lflag &=~ (ICANON | ECHO);
        // Setting a new line, then end of file
        raw.c_cc[VEOL] = 1;
        raw.c_cc[VEOF] = 2;
        tcsetattr(kfd, TCSANOW, &raw);
    #endif
}
void readOne(char * c)
{
    #ifndef _WIN32
        int rc = read(kfd, c, 1);
        if (rc < 0)
        {
            throw std::runtime_error("read failed");
        }
    #endif
}
void shutdown()
{
    #ifndef _WIN32
        tcsetattr(kfd, TCSANOW, &cooked);
    #endif
    }
    private:
    #ifndef _WIN32
    int kfd;
    struct termios cooked;
    #endif
};

KeyboardReader input;

class KeyboradControl
{
    public:
    KeyboradControl();
    void keyLoop();

    private:


    ros::NodeHandle nh_;
    double x_, y_, z_, x_scale_, y_scale_, z_scale_;
    double height_, pitch_, yaw_, height_scale_, pitch_scale_, yaw_scale_;
    ros::Publisher twist_pub_;
    ros::Publisher gimbal_pub_;

};

    KeyboradControl::KeyboradControl():
    x_(0), y_(0), z_(0), x_scale_(2.0), y_scale_(2.0), z_scale_(2.0),
    height_(0), pitch_(0), yaw_(0)
    {
        twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        gimbal_pub_ = nh_.advertise<geometry_msgs::Vector3>("cmd_pos", 1);
    }


    


void quit(int sig)
{
    (void)sig;
    input.shutdown();
    ros::shutdown();
    exit(0);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "keyboard_control");
    KeyboradControl keyboard_control_;

    signal(SIGINT,quit);

    keyboard_control_.keyLoop();
    quit(0);

    return(0);
}


void KeyboradControl::keyLoop()
{
    char c;
    bool dirty = false;

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move engineer. '`' to quit.");


    for(;;)
    {
        // get the next event from the keyboard  
        try
        {
            input.readOne(&c);
        }
        catch (const std::runtime_error &)
        {
            perror("read():");
            return;
        }

        x_ = y_= z_ = 0;

        switch(c)
        {
            case 'A':
            case 'a':
                y_ = 1.0;
                dirty = true;
                break;
            case 'D':
            case 'd':
                ROS_INFO_STREAM("RIGHT");
                y_ = -1.0;
                dirty = true;
                break;
            case 'W':
            case 'w':
                ROS_INFO_STREAM("UP");
                x_ = 1.0;
                dirty = true;
                break;
            case 'S':
            case 's':
                ROS_INFO_STREAM("DOWN");
                x_ = -1.0;
                dirty = true;
                break;
            case 'Q':
            case 'q':
                ROS_INFO_STREAM("COUNTERCLOCKWISE");
                z_ = 1.0;
                dirty = true;
                break;
            case 'E':
            case 'e':
                ROS_INFO_STREAM("CLOCKWISE");
                z_ = -1.0;
                dirty = true;
                break;
            case '5':
                ROS_INFO_STREAM("PITCH UP");
                pitch_ += 0.05;
                dirty = true;
                break;
            case '2':
                ROS_INFO_STREAM("PITCH DOWN");
                pitch_ -= 0.05;
                dirty = true;
                break;
            case '1':
                ROS_INFO_STREAM("YAW UP");
                yaw_ += 0.05;
                dirty = true;
                break;
            case '3':
                ROS_INFO_STREAM("YAW DOWN");
                yaw_ -= 0.05;
                dirty = true;
                break;
            case '4':
                ROS_INFO_STREAM("HEIGHT UP");
                height_ += 0.005;
                dirty = true;
                break;
            case '6':
                ROS_INFO_STREAM("HEIGHT DOWN");
                height_ -= 0.005;
                dirty = true;
                break;
            case '`':
                ROS_INFO_STREAM("QUIT");
                return;
        }


        geometry_msgs::Twist twist;
        twist.linear.x = x_scale_*x_;
        twist.linear.y = y_scale_*y_;
        twist.angular.z = z_scale_*z_;
        geometry_msgs::Vector3 pos;
        pos.x = yaw_;
        pos.y = pitch_;
        pos.z = height_;
        if(dirty == true)
        {
            twist_pub_.publish(twist);  
            gimbal_pub_.publish(pos);  
            dirty = false;
        }
    }


    return;
}
