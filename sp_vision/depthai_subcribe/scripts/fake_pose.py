import rospy
from geometry_msgs.msg import Pose
import numpy as np

def talker():
    rospy.init_node('fake', anonymous=True)
    pub = rospy.Publisher('calibrate', Pose, queue_size=10)
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        vel_msg = Pose()
        vel_msg.position.x = 1
        vel_msg.position.y = 1
        vel_msg.position.z = 1
        vel_msg.orientation.w = 1.0
        vel_msg.orientation.w = 0.0
        vel_msg.orientation.w = -1.0
        vel_msg.orientation.w = 0.4
        pub.publish(vel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass