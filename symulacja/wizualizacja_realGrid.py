#!/usr/bin/env python
import rospy
import numpy as np
from irpos import *
from std_msgs.msg import *
from serwo.msg import SerwoInfo
import threading
import PyKDL
import matrixOperations

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
import math

T_gC_vector = []
pointInCheckerFrame = np.matrix('0.0;0.0;0.0;1.0')
newPoint = [0,0,0,0]
realHomogMatrix = np.zeros(shape=(4,4))
T_bC_matrix = np.zeros(shape=(4,4))
#T_bC_matrix = np.empty([4, 4])
publisher = None
foundLock = threading.Lock()
homogLock = threading.Lock()

def readHomogVector(msg):
	homogLock.acquire()
	global T_gC_vector
	T_gC_vector = msg.matrix
	homogLock.release()

def calculateNewPoint():
	global newPoint
	newPoint = realHomogMatrix * pointInCheckerFrame
        #print newPoint
	
def onInit():
	rospy.Subscriber('realHomogMatrix',SerwoInfo, readHomogVector)
	topic = 'realGrid'
	global publisher
	publisher = rospy.Publisher(topic, Marker,queue_size=10)
	rospy.init_node('realGrid')
	
        
def realGrid():
	rate = rospy.Rate(500)
	pawn = Marker()
	pawn.header.frame_id = "/world"
	pawn.type = pawn.CUBE
	pawn.action = pawn.ADD

        listener = tf.TransformListener()
        listener.waitForTransform('/p_c_optical_frame', '/world', rospy.Time(0), rospy.Duration(10))
        #T_bC pozycja /wordl (B) w ukladzie /p_c_optical_frame (C)
    	(trans,rot) = listener.lookupTransform('/p_c_optical_frame', '/world', rospy.Time(0))
	T_bC = matrixOperations.euler_matrix_from_quaternion(rot, trans)

        T_gC = matrixOperations.translation_from_vector(T_gC_vector)

	T_gB = np.dot(np.linalg.inv(T_bC) , T_gC)
  	print T_gB
        global realHomogMatrix
	realHomogMatrix = T_gB

	T = PyKDL.Frame(PyKDL.Rotation(realHomogMatrix[0][0], realHomogMatrix[0][1], realHomogMatrix[0][2],realHomogMatrix[1][0], realHomogMatrix[1][1], realHomogMatrix[1][2], realHomogMatrix[2][0], realHomogMatrix[2][1], realHomogMatrix[2][2]))
    
	q = T.M.GetQuaternion()
	pawn.pose = Pose(Point(newPoint[0],newPoint[1],newPoint[2]), Quaternion(q[0],q[1],q[2],q[3]))
	pawn.color.a = 1.0
	pawn.color.r = 1.0
	pawn.color.g = 1.0
	pawn.color.b = 0.0
	pawn.scale.x = 0.2
	pawn.scale.y = 0.4
	pawn.scale.z = 0.01
		
	pawn.scale.x = 0.2
	pawn.scale.y = 0.4
	pawn.scale.z = 0.01
		
	publisher.publish(pawn)
	rate.sleep()
	#rospy.sleep(0.002)

if __name__ == '__main__':
	onInit()
	while not rospy.is_shutdown():
		calculateNewPoint()
		realGrid()
