#include <ros/ros.h>
#include <sp_common/DbusData.h>
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
    ros::Publisher dbus_data_pub_;
    sp_common::DbusData dbus_data_;

};

KeyboradControl::KeyboradControl()
{
    dbus_data_pub_ = nh_.advertise<sp_common::DbusData>("/dbus_data", 1);
    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move engineer. '`' to quit.");
}

  


void quit(int sig)
{
    (void)sig;
    input.shutdown();
    ros::shutdown();
    exit(0);
}



void KeyboradControl::keyLoop()
{
    char c;
    bool dirty = false;


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
    printf("char: %c\n" ,c);
    dbus_data_.ch_l_x = 0.0;
    dbus_data_.ch_l_y = 0.0;
    dbus_data_.ch_r_x = 0.0;
    dbus_data_.ch_r_y = 0.0;
    dbus_data_.m_x = 0.0;
    dbus_data_.m_y = 0.0;
    dbus_data_.m_z = 0.0;
    dbus_data_.p_l = false;
    dbus_data_.p_r = false;
    dbus_data_.key_q = false;
    dbus_data_.key_w = false;
    dbus_data_.key_e = false;
    dbus_data_.key_r = false;
    dbus_data_.key_a = false;
    dbus_data_.key_s = false;
    dbus_data_.key_d = false;
    dbus_data_.key_f = false;
    dbus_data_.key_g = false;
    dbus_data_.key_z = false;
    dbus_data_.key_x = false;
    dbus_data_.key_c = false;
    dbus_data_.key_v = false;
    dbus_data_.key_b = false;
    dbus_data_.key_shift = false;
    dbus_data_.key_ctrl = false;


    switch(c)
    {
        case 'A':
        case 'a':
            dbus_data_.ch_r_y = 1.0;
            dbus_data_.key_a = true;
            break;
        case 'D':
        case 'd':
            dbus_data_.ch_r_y = -1.0;
            dbus_data_.key_d = true;
            break;
        case 'W':
        case 'w':
            dbus_data_.ch_r_x = 1.0;
            dbus_data_.key_w = true;
            break;
        case 'S':
        case 's':
            dbus_data_.ch_r_x = -1.0;
            dbus_data_.key_s = true;
            break;
        case 'Q':
        case 'q':
            dbus_data_.key_q = true;
            break;
        case 'E':
        case 'e':
            dbus_data_.key_e = true;
            break;
        case 'R':
        case 'r':
            dbus_data_.key_r = true;
            break;
        case 'F':
        case 'f':
            dbus_data_.key_f = true;
            break;
        case 'G':
        case 'g':
            dbus_data_.key_g = true;
            break;
        case 'Z':
        case 'z':
            dbus_data_.key_z = true;
            break;
        case 'X':
        case 'x':
            dbus_data_.key_x = true;
            break;
        case 'C':
        case 'c':
            dbus_data_.key_c = true;
            break;
        case 'V':
        case 'v':
            dbus_data_.key_v = true;
            break;
        case 'B':
        case 'b':
            dbus_data_.key_b = true;
            break;
        case 'I':
        case 'i':
            dbus_data_.m_y = 1.0;
            break;
        case 'K':
        case 'k':
            dbus_data_.m_y = -1.0;
            break;
        case 'J':
        case 'j':
            dbus_data_.m_x = 1.0;
            break;
        case 'L':
        case 'l':
            dbus_data_.m_x = -1.0;
        case 'U':
        case 'u':
            dbus_data_.p_l = true;
            break;
        case 'O':
        case 'o':
            dbus_data_.p_r = true;
            break;
        case '3':
            dbus_data_.s_r = 2;
            break;
        case '6':
            dbus_data_.s_r = 3;
            break;
        case '9':
            dbus_data_.s_r = 1;
            break;
        case '1':
            dbus_data_.s_l = 2;
            break;
        case '4':
            dbus_data_.s_l = 3;
            break;
        case '7':
            dbus_data_.s_l = 1;
            break;
        case '`':
            ROS_INFO_STREAM("QUIT");
            return;
    }
    dbus_data_pub_.publish(dbus_data_);  

}
