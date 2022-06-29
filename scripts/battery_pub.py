#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import BatteryState

def callback(data):
    global pub_battery
    battery_percentage = data.percentage
    pub_battery.publish(battery_percentage)
    
    
def listener():
    global pub_battery
    rospy.init_node('battery_tmp_sub', anonymous=True)
    rospy.Subscriber("battery", BatteryState, callback)
    pub_battery = rospy.Publisher('battery_percentage', Float32, queue_size=10)
    
    rospy.spin()

if __name__ == '__main__':
    listener()
