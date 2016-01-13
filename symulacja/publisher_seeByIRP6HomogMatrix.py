#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from serwo.msg import SerwoInfo

def talker():
    pub = rospy.Publisher('seeByIRP6HomogMatrix', SerwoInfo, queue_size=10)
    rospy.init_node('seeByIRP6HomogMatrix', anonymous=True)
    msg = SerwoInfo()
    matrix = []

    #w rzeczywistosci obiekt poruszasza sie z taka czestotliwoscia
    rate = rospy.Rate(40) 
    while not rospy.is_shutdown():
        y = math.sin(rospy.get_time()/5)
        y = 2.2 + y/4
        matrix = [1,0,0,0,0,1,0,y,0,0,1,0,0,0,0,1]
        msg.matrix = matrix;
        msg.found = 1;
        msg.out_time_nsec_pocz = 0;
        msg.out_time_sec_pocz = 0;
        msg.out_time_nsec_kon = 0;
        msg.out_time_sec_kon = 0;
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
