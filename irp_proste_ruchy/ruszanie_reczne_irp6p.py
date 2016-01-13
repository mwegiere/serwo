#!/usr/bin/env python


#skrypt sluzy do poruszaninia robotrm irp-6 z klawiatury

from irpos import *

import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import tf
import numpy as np


class image_converter:

  def __init__(self):
    self.running = False
    
    cv2.namedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/image_color", Image, self.callback)
    
    self.irpos = IRPOS("IRpOS", "Irp6p", 6, "irp6p_manager")
    #self.images = []
    
  def interpret_key(delf, key, key_up, key_down, axis, pos, vel):
    if key == key_up:
      pos[axis] = pos[axis] + vel[axis] * 0.2
    if key == key_down:
      pos[axis] = pos[axis] - vel[axis] * 0.2
      
    return pos
    
  def step_move(self):
    max_vel = [0.5, 0.5, 0.5, 0.5, 0.5, 0.25]
    
    rospy.sleep(2)
    
    key = -1
    while key != 27:
      pos_current = self.irpos.get_joint_position()
      pos_current_cartesian = self.irpos.get_cartesian_pose()
      pos_current = list(pos_current)
      old_pos = list(pos_current)
      key = cv2.waitKey(10)
      key = key & 255
      
      pos_current = self.interpret_key(chr(key), 'q', 'a', 0, pos_current, max_vel)
      pos_current = self.interpret_key(chr(key), 'w', 's', 1, pos_current, max_vel)
      pos_current = self.interpret_key(chr(key), 'e', 'd', 2, pos_current, max_vel)
      pos_current = self.interpret_key(chr(key), 'r', 'f', 3, pos_current, max_vel)
      pos_current = self.interpret_key(chr(key), 't', 'g', 4, pos_current, max_vel)
      pos_current = self.interpret_key(chr(key), 'y', 'h', 5, pos_current, max_vel)
      
      if chr(key) == 'p':
        print pos_current

      if chr(key) == 'k':
        print pos_current_cartesian

      if pos_current != old_pos:
        self.irpos.move_to_joint_position(pos_current, 1)
          
  def synchro(self):
    self.irpos.move_to_synchro_position(15)

  def callback(self, data):
      
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
     
      if self.running:
        self.images.append(data)
        print len(self.images)

      cv2.imshow("Image window", cv_image)
      cv2.waitKey(3)
      
    except Exception, e:
      print e


def main(args):
  ic = image_converter()
  ic.step_move()
  #ic.synchro()
  
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

