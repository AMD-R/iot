#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32

cur_vel = 0
        
def callback(data):
    global lim_vel
    global cur_vel
    cur_vel = data
    
def listener():
    global lim_vel
    global cur_vel
    rospy.init_node('iot_vel_pub', anonymous=True)
    rospy.Subscriber("wheel_vel_l", Float32, callback)
    lim_vel = rospy.Publisher('iot/velocity', Float32, queue_size=10)
    rate = rospy.Rate(0.1)
    while not rospy.is_shutdown():
        lim_vel.publish(cur_vel)
        print(cur_vel)
        rate.sleep()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

