#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix
from gps import *

def talker():
    pub = rospy.Publisher('iot/gps', NavSatFix, queue_size=10) #gpspos = topic name, String = msg type
    rospy.init_node('gpspub', anonymous=True) #name of the node
    rate = rospy.Rate(0.2) #0.2Hz = every 5 seconds
    
    gpsd = gps(mode=WATCH_ENABLE|WATCH_NEWSTYLE)
    while not rospy.is_shutdown():
        nx = gpsd.next()
        if nx['class'] == 'TPV':
            latitude = getattr(nx,'lat', "Unknown") #dble
            longitude = getattr(nx,'lon', "Unknown")
            pos = NavSatFix()
            pos.longitude = longitude
            pos.latitude = latitude
            rospy.loginfo(pos)
            pub.publish(pos)
            
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

