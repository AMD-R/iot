#!/usr/bin/env python3
# call/home button to adafruit

import rospy
from std_msgs.msg import String
#from sensor_msgs.msg import BatteryState

def callback(data):
    global pub_battery
    order = data
    pub_order.publish(order)
    
def listener():
    global pub_order
    rospy.init_node('function_adafruit', anonymous=True)
    rospy.Subscriber("order", String, callback)
    pub_order = rospy.Publisher('order', String, queue_size=10)
    
    rospy.spin()

if __name__ == '__main__':
    listener()