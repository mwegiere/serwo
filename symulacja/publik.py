#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from std_msgs.msg import Int64
import time

def talker():
    pub = rospy.Publisher('uchyb', Int64, queue_size=10)
    rospy.init_node('uchyb', anonymous=True)
    msg = Int64()

    rate = rospy.Rate(500) 
    while not rospy.is_shutdown():

	t = rospy.Time.from_sec(time.time())
	seconds = t.to_sec() #floating point
	nanoseconds = t.to_nsec()
	#print nanoseconds
        msg.data = nanoseconds;
        pub.publish(msg)
        rate.sleep()	
        #print time.time().nsec





if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
