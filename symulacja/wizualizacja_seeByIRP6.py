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

y = 0
homogVector = []
pointInCheckerFrame = np.matrix('0.0;0.0;0.0;1.0')
newPoint = [0,0,0,0]
seeByIRP6HomogMatrix = np.zeros(shape=(4,4))
publisher = None
homogLock = threading.Lock()

def readHomogVector(msg):
	homogLock.acquire()
	global y
	y = msg.data
	global seeByIRP6HomogMatrix
	seeByIRP6HomogMatrix = np.array([[1,0,0,0.9],[0,1,0,2.2+y/4],[0,0,1,0.7],[0,0,0,1]])
	homogLock.release()

def calculateNewPoint():
	global newPoint
	newPoint = seeByIRP6HomogMatrix * pointInCheckerFrame
	
def onInit():
	rospy.Subscriber('seeByIRP6HomogMatrix',Float32, readHomogVector)
	topic = 'seeByIRP6Grid'
	global publisher
	publisher = rospy.Publisher(topic, Marker,queue_size=10)
	rospy.init_node('seeByIRP6Grid')
	
def seeByIRP6Grid():
	pawn = Marker()
	pawn.header.frame_id = "/world"
	pawn.type = pawn.CUBE
	pawn.action = pawn.ADD
	
	global seeByIRP6HomogMatrix
	T = PyKDL.Frame(PyKDL.Rotation(seeByIRP6HomogMatrix[0][0], seeByIRP6HomogMatrix[0][1], seeByIRP6HomogMatrix[0][2],seeByIRP6HomogMatrix[1][0], seeByIRP6HomogMatrix[1][1], seeByIRP6HomogMatrix[1][2], seeByIRP6HomogMatrix[2][0], seeByIRP6HomogMatrix[2][1], seeByIRP6HomogMatrix[2][2]))
    
	q = T.M.GetQuaternion()
	pawn.pose = Pose(Point(newPoint[0],newPoint[1],newPoint[2]), Quaternion(q[0],q[1],q[2],q[3]))
	pawn.color.a = 1.0
	pawn.color.r = 1.0
	pawn.color.g = 0.0
	pawn.color.b = 0.0
	pawn.scale.x = 0.2
	pawn.scale.y = 0.4
	pawn.scale.z = 0.01
	
	
	pawn.scale.x = 0.2
	pawn.scale.y = 0.4
	pawn.scale.z = 0.01
		
	publisher.publish(pawn)
	rospy.sleep(0.6)

if __name__ == '__main__':
	onInit()
	while not rospy.is_shutdown():
		calculateNewPoint()
		seeByIRP6Grid()
