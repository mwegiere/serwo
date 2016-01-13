#!/usr/bin/env python

#This node publishes error in camera coordination frame

import rospy
import math
from serwo.msg import ErrorInfo

def talker():
    pub = rospy.Publisher('error', ErrorInfo, queue_size=10)
    rospy.init_node('error_generator', anonymous=True)
    msg = ErrorInfo()
    rate = rospy.Rate(500) 

    while not rospy.is_shutdown():       
        y = math.sin(rospy.get_time()/5)
        error = y/5    
        msg.error = error
        msg.found = 1
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
