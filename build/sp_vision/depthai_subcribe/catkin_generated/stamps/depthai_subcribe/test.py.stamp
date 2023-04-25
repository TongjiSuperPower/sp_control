import rospy
from std_msgs.msg import Int32
import  sys
import  tty, termios

if __name__ == "__main__":
    #2.初始化 ROS 节点:命名(唯一)
    rospy.init_node("talker_p")
    #3.实例化 发布者 对象
    pub = rospy.Publisher("chatter",Int32,queue_size=10)
    #4.组织被发布的数据，并编写逻辑发布数据
    msg = Int32()  #创建 msg 对象
    # msg_front = "hello 你好"
    # count = 0  #计数器 
    # 设置循环频率
    rate = rospy.Rate(1)
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
            msg.data = 3000
        # if ch == 'o':
        #     msg.data = 20000
        elif ch == 'q':
            exit()

        # else:
        #     msg.data = 15000

        pub.publish(msg)
        rate.sleep()
        rospy.loginfo("写出的数据:%s",msg.data)
