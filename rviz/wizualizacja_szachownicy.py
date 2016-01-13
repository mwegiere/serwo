#!/usr/bin/env python
import rospy
import numpy as np
from irpos import *
from std_msgs.msg import *
import threading
import PyKDL

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
import math

homogVector = []
pointInCheckerFrame = np.matrix('0.0;0.0;0.0;1.0')
newPoint = [0,0,0,0]
homogMatrix = np.zeros(shape=(4,4))
publisher = None
homogLock = threading.Lock()
found = 0
foundLock = threading.Lock()

def readHomogVector(msg):
	homogLock.acquire()
	global homogVector
	homogVector = msg.data
	global homogMatrix
	homogMatrix = np.array([[homogVector[0],homogVector[1],homogVector[2],homogVector[3]],[homogVector[4],homogVector[5],homogVector[6],homogVector[7]],[homogVector[8],homogVector[9],homogVector[10],homogVector[11]],[homogVector[12],homogVector[13],homogVector[14],homogVector[15]]])
	homogLock.release()

def readFound(msg):
	foundLock.acquire()
	global found
	found = msg.data
	foundLock.release()

def calculateNewPoint():
	global newPoint
	newPoint = homogMatrix * pointInCheckerFrame
	#print newPoint 
	
def onInit():
	rospy.Subscriber('/homog_matrix',Float32MultiArray, readHomogVector)
	#rospy.Subscriber('/found',Int, readFound)
	topic = 'pawn_visualization'
	global publisher
	publisher = rospy.Publisher(topic, Marker,queue_size=10)
	rospy.init_node('register')
	
def pawnCreation():
	pawn = Marker()
	pawn.header.frame_id = "/p_c_optical_frame"
	pawn.type = pawn.CUBE
	pawn.action = pawn.ADD
	
	global homogMatrix
	T = PyKDL.Frame(PyKDL.Rotation(homogMatrix[0][0], homogMatrix[0][1], homogMatrix[0][2],homogMatrix[1][0], homogMatrix[1][1], homogMatrix[1][2], homogMatrix[2][0], homogMatrix[2][1], homogMatrix[2][2]))
	print homogMatrix
    
	q = T.M.GetQuaternion()
	pawn.pose = Pose(Point(newPoint[0],newPoint[1],newPoint[2]), Quaternion(q[0],q[1],q[2],q[3]))
	pawn.color.a = 1.0
	pawn.color.r = 1.0
	pawn.color.g = 1.0
	pawn.color.b = 0.0
	pawn.scale.x = 0.2
	pawn.scale.y = 0.4
	pawn.scale.z = 0.01
	
	

	if homogMatrix[0][0] != 0.0 :
		pawn.scale.x = 0.2
		pawn.scale.y = 0.4
		pawn.scale.z = 0.01
	
	else :
		pawn.scale.x = 0.0
		pawn.scale.y = 0.0
		pawn.scale.z = 0.0
	
	publisher.publish(pawn)
	rospy.sleep(0.01)

if __name__ == '__main__':
	onInit()
	while not rospy.is_shutdown():
		calculateNewPoint()
		pawnCreation()
