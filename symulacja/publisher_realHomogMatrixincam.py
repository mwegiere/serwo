#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from serwo.msg import SerwoInfo
import tf
from tf import transformations
import numpy as np
from numpy.linalg import inv
import matrixOperations

def talker():
    pub = rospy.Publisher('realHomogMatrix', SerwoInfo, queue_size=10)
    rospy.init_node('realHomogMatrix', anonymous=True)
    msg = SerwoInfo()
    vector_gB = []
    vector_gC = []
    listener = tf.TransformListener()
    a = np.empty([4, 4])
    b = np.empty([4, 4])
    rate = rospy.Rate(500) 
    listener.waitForTransform('/p_c_optical_frame', '/world', rospy.Time(0), rospy.Duration(10))

    while not rospy.is_shutdown():       
        #T_bC pozycja /wordl (B) w ukladzie /p_c_optical_frame (C)
    	(trans,rot) = listener.lookupTransform('/p_c_optical_frame', '/world', rospy.Time(0))
	T_bC = matrixOperations.euler_matrix_from_quaternion(rot, trans)

        y = math.sin(rospy.get_time()/5)
        y = 2.3 + y/5
        #x = math.sin(rospy.get_time()/5)
        #x = 1 + y/2
        vector_gB = [1,0,0,0.88,0,1,0,0,4,0,1,0.6,0,5,0,1]
	vector_gB = [1,0,0,0.88, 0,1,0,0, 0,0,1,0.6, 0,0,0,1]
	T_gB = matrixOperations.translation_from_vector(vector_gB)
        a = T_gB

	T_gC = np.dot(T_bC , T_gB)

        vector_gC = matrixOperations.vector_from_translation(T_gC)
     
        #msg.matrix = vector_gC;
	msg.matrix = vector_gB;
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
