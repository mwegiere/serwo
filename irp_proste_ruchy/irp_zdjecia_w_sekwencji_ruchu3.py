#!/usr/bin/env python

#T_AB - przeksztalcenie z ukladu A do B, czyli pozycja ukladu B w ukladzie A
#D - uklad diod
#C - uklad optyczny kamery
#W - uklad swiata

#Tq_AB - kwaternion przeksztalcenia z ukladu A do B
#Tt_AB - translacja przeksztalcenia z ukladu A do B
import os, sys
from irpos import *

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

import time

import matrixOperations

#pakiety sluzace do dynamicznej rekonfiguracji parametrow kamery
PACKAGE = 'pointgrey_camera_driver'
import roslib;roslib.load_manifest(PACKAGE)
import rospy
import dynamic_reconfigure.client

#pakiety do subskrybcji tf
import roslib
roslib.load_manifest('serwo')
import rospy
import math
import tf
from tf import transformations
import numpy
import geometry_msgs.msg

class image_converter:

  def fromTranslationRotation(self, translation, rotation):
    return numpy.dot(transformations.translation_matrix(translation), transformations.quaternion_matrix(rotation))

  def __init__(self): 
    cv2.namedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/image_color", Image, self.makePhotoCallback) 
    self.irpos = IRPOS("IRpOS", "Irp6p", 6, "irp6p_manager")
    self.goodImages = []
    self.image = None
 
  def run(self):
    homogenous = np.array([[0,0,0,2],[0,3,4,4],[5,5,6,3],[0,0,0,1]])
    self.irpos.move_to_joint_position([0.0, -1.57079632679, -0.0, -0.0, 4.71238898038, 1.57079632679], 10.0)
    self.irpos.set_tool_geometry_params(Pose(Point(0.0, 0.0, 0.5), Quaternion(0.0, 0.0, 0.0, 1.0)))

    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    rate.sleep()
    
    for i in range(71):
 
       self.irpos.move_rel_to_cartesian_pose(1.0,Pose(Point(0.0, 0.0, 0.0), Quaternion(-0.00872654, 0.0, 0.0, 0.99996192)))
       
  def makePhotoCallback(self, data):  
    try:  
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      self.image = data   
      cv2.imshow("Image window", cv_image)
      cv2.waitKey(3)      
    except Exception, e:
      print e

  def dynamicReconfigureCallback(self,config):
    rospy.loginfo("Config set to {shutter_speed}".format(**config))
    rospy.loginfo("Config set to {gain}".format(**config))
    rospy.loginfo("Config set to {frame_rate}".format(**config))

def main(args):
  ic = image_converter()
  ic.run()
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)


