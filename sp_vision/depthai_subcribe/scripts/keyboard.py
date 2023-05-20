import  sys
import  tty, termios
import rospy
from std_msgs.msg import Int32

# 全局变量
msg = Int32()
pub = rospy.Publisher('expTime',Int32,queue_size=1)
def keyboardLoop():
    #初始化
    rospy.init_node('OAK_teleop')
    # Set rospy to exectute a shutdown function when exiting       
    rate = rospy.Rate(1)
    #显示提示信息
    print("Reading from keyboard")
    print("Use i keys to control the exptime")
    print("Press q to quit")
    #读取按键循环
    while not rospy.is_shutdown():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
		#不产生回显效果
        old_settings[3] = old_settings[3] & ~termios.ICANON & ~termios.ECHO
        try :
            tty.setraw( fd )
            ch = sys.stdin.read( 1 )
        finally :
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        if ch == 'i':
            msg.data = 2000
        if ch == 'o':
            msg.data = 20000
        if ch == 'q':
            exit()
        pub.publish(msg) 
        rate.sleep()

if __name__ == '__main__':
    try:
        keyboardLoop()
    except rospy.ROSInterruptException:
        pass
