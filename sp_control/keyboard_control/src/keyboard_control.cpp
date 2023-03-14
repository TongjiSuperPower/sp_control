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


#define KEYCODE_LEFT 0x25 
#define KEYCODE_UP 0x26 
#define KEYCODE_RIGHT 0x27
#define KEYCODE_DOWN 0x28 
#define KEYCODE_ESCAPE 0x18
#define KEYCODE_A 0x41
#define KEYCODE_B 0x42
#define KEYCODE_C 0x43
#define KEYCODE_D 0x44
#define KEYCODE_E 0x45
#define KEYCODE_F 0x46
#define KEYCODE_G 0x47
#define KEYCODE_H 0x48
#define KEYCODE_I 0x49
#define KEYCODE_J 0x4A
#define KEYCODE_K 0x4B
#define KEYCODE_L 0x4C
#define KEYCODE_M 0x4D
#define KEYCODE_N 0x4E
#define KEYCODE_O 0x4F
#define KEYCODE_P 0x50
#define KEYCODE_Q 0x51
#define KEYCODE_R 0x52
#define KEYCODE_S 0x53
#define KEYCODE_T 0x54
#define KEYCODE_U 0x55
#define KEYCODE_V 0x56
#define KEYCODE_W 0x57
#define KEYCODE_X 0x58
#define KEYCODE_Y 0x59
#define KEYCODE_Z 0x5A

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
    #else
    for(;;)
    {
        HANDLE handle = GetStdHandle(STD_INPUT_HANDLE);
        INPUT_RECORD buffer;
        DWORD events;
        PeekConsoleInput(handle, &buffer, 1, &events);
        if(events > 0)
        {
            ReadConsoleInput(handle, &buffer, 1, &events);
            if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_LEFT)
            {
                *c = KEYCODE_LEFT;
                return;
            }
            else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_UP)
            {
                *c = KEYCODE_UP;
                return;
            }
            else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_RIGHT)
            {
                *c = KEYCODE_RIGHT;
                return;
            }
            else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_DOWN)
            {
                *c = KEYCODE_DOWN;
                return;
            }
            else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_OEM_COMMA)
            {
                *c = KEYCODE_COUNTERCLOCKWISE;
                return;
            }
            else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_OEM_PERIOD)
            {
                *c = KEYCODE_CLOCKWISE;
                return;
            }
            else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_ESCAPE)
            {
                *c = KEYCODE_ESCAPE;
                return;
            }
            else if (buffer.Event.KeyEvent.wVirtualKeyCode == 'A'||buffer.Event.KeyEvent.wVirtualKeyCode == 'a')
            {
                *c = KEYCODE_A;
                return;
            }
            else if (buffer.Event.KeyEvent.wVirtualKeyCode == 'B')
            {
                *c = KEYCODE_B;
                return;
            }
            else if (buffer.Event.KeyEvent.wVirtualKeyCode == 'C')
            {
                *c = KEYCODE_C;
                return;
            }
            else if (buffer.Event.KeyEvent.wVirtualKeyCode == 'D'||buffer.Event.KeyEvent.wVirtualKeyCode == 'd')
            {
                *c = KEYCODE_D;
                return;
            }
            else if (buffer.Event.KeyEvent.wVirtualKeyCode == 'E'||buffer.Event.KeyEvent.wVirtualKeyCode == 'e')
            {
                *c = KEYCODE_E;
                return;
            }
            else if (buffer.Event.KeyEvent.wVirtualKeyCode == 'F')
            {
                *c = KEYCODE_F;
                return;
            }
            else if (buffer.Event.KeyEvent.wVirtualKeyCode == 'G')
            {
                *c = KEYCODE_G;
                return;
            }
            else if (buffer.Event.KeyEvent.wVirtualKeyCode == 'H')
            {
                *c = KEYCODE_H;
                return;
            }
            else if (buffer.Event.KeyEvent.wVirtualKeyCode == 'I')
            {
                *c = KEYCODE_I;
                return;
            }
            else if (buffer.Event.KeyEvent.wVirtualKeyCode == 'J')
            {
                *c = KEYCODE_J;
                return;
            }
            else if (buffer.Event.KeyEvent.wVirtualKeyCode == 'K')
            {
                *c = KEYCODE_K;
                return;
            }
            else if (buffer.Event.KeyEvent.wVirtualKeyCode == 'L')
            {
                *c = KEYCODE_L;
                return;
            }
            else if (buffer.Event.KeyEvent.wVirtualKeyCode == 'M')
            {
                *c = KEYCODE_M;
                return;
            }
            else if (buffer.Event.KeyEvent.wVirtualKeyCode == 'N')
            {
                *c = KEYCODE_N;
                return;
            }
            else if (buffer.Event.KeyEvent.wVirtualKeyCode == 'O')
            {
                *c = KEYCODE_O;
                return;
            }
            else if (buffer.Event.KeyEvent.wVirtualKeyCode == 'P')
            {
                *c = KEYCODE_P;
                return;
            }
            else if (buffer.Event.KeyEvent.wVirtualKeyCode == 'Q'||buffer.Event.KeyEvent.wVirtualKeyCode == 'q')
            {
                *c = KEYCODE_Q;
                return;
            }
            else if (buffer.Event.KeyEvent.wVirtualKeyCode == 'R')
            {
                *c = KEYCODE_R;
                return;
            }
            else if (buffer.Event.KeyEvent.wVirtualKeyCode == 'S'||buffer.Event.KeyEvent.wVirtualKeyCode == 's')
            {
                *c = KEYCODE_S;
                return;
            }
            else if (buffer.Event.KeyEvent.wVirtualKeyCode == 'T')
            {
                *c = KEYCODE_T;
                return;
            }
            else if (buffer.Event.KeyEvent.wVirtualKeyCode == 'U')
            {
                *c = KEYCODE_U;
                return;
            }
            else if (buffer.Event.KeyEvent.wVirtualKeyCode == 'V')
            {
                *c = KEYCODE_V;
                return;
            }
            else if (buffer.Event.KeyEvent.wVirtualKeyCode == 'W'||buffer.Event.KeyEvent.wVirtualKeyCode == 'w')
            {
                *c = KEYCODE_W;
                return;
            }
            else if (buffer.Event.KeyEvent.wVirtualKeyCode == 'X')
            {
                *c = KEYCODE_X;
                return;
            }
            else if (buffer.Event.KeyEvent.wVirtualKeyCode == 'Y')
            {
                *c = KEYCODE_Y;
                return;
            }
            else if (buffer.Event.KeyEvent.wVirtualKeyCode == 'Z')
            {
                *c = KEYCODE_Z;
                return;
            }
        }
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
    ros::Publisher twist_pub_;

};

    KeyboradControl::KeyboradControl():
    x_(0),
    y_(0),
    z_(0),
    x_scale_(2.0),
    y_scale_(2.0),
    z_scale_(2.0)
    {
        twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
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
    ros::init(argc, argv, "keyborad_control");
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
    puts("Use arrow keys to move engineer. 'esc' to quit.");


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

        x_=y_=z_=0;
        ROS_DEBUG("value: 0x%02X\n", c);

        switch(c)
        {
        case KEYCODE_LEFT:
        case KEYCODE_A:
            ROS_DEBUG("LEFT");
            y_ = 1.0;
            dirty = true;
            break;
        case KEYCODE_RIGHT:
        case KEYCODE_D:
            ROS_DEBUG("RIGHT");
            y_ = -1.0;
            dirty = true;
            break;
        case KEYCODE_UP:
        case KEYCODE_W:
            ROS_DEBUG("UP");
            x_ = 1.0;
            dirty = true;
            break;
        case KEYCODE_DOWN:
        case KEYCODE_S:
            ROS_DEBUG("DOWN");
            x_ = -1.0;
            dirty = true;
            break;
        case KEYCODE_N:
        case KEYCODE_Q:
            ROS_DEBUG("COUNTERCLOCKWISE");
            z_ = 1.0;
            dirty = true;
            break;
        case KEYCODE_M:
        case KEYCODE_E:
            ROS_DEBUG("CLOCKWISE");
            z_ = -1.0;
            dirty = true;
            break;
        
        case KEYCODE_ESCAPE:
            ROS_DEBUG("QUIT");
            return;
        }


        geometry_msgs::Twist twist;
        twist.linear.x = x_scale_*x_;
        twist.linear.y = y_scale_*y_;
        twist.angular.z = z_scale_*z_;
        if(dirty == true)
        {
            twist_pub_.publish(twist);    
            dirty = false;
        }
    }


    return;
}
