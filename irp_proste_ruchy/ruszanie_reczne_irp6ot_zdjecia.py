#!/usr/bin/env python
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
    
    self.irpos = IRPOS("IRpOS", "Irp6ot", 7, "irp6ot_manager")
    #self.listener = tf.TransformListener(True, rospy.Duration(100))
    self.images = []
    #self.trs = []
    
  def interpret_key(delf, key, key_up, key_down, axis, pos, vel):
    if key == key_up:
      pos[axis] = pos[axis] + vel[axis] * 0.2
    if key == key_down:
      pos[axis] = pos[axis] - vel[axis] * 0.2
      
    return pos
    
  def step_move(self):
    max_vel = [ 0.05, 0.5, 0.5, 0.5, 0.5, 0.5, 0.25]
    
    rospy.sleep(2)
    
    key = -1
    while key != 27:
      pos_current = self.irpos.get_joint_position()
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
      pos_current = self.interpret_key(chr(key), 'u', 'j', 6, pos_current, max_vel)
      
      if chr(key) == 'p':
        print pos_current
        
      if chr(key) == '1':
        self.irpos.move_to_joint_position([0.21067300000000005, 0.5876247031931617, -1.671971372180727, -0.27887683340132824, 0.5204890924180978, 4.130689120591732, 0.21102631103562178], 15)
      if chr(key) == '2':
        self.irpos.move_to_joint_position([0.21067300000000005, 0.3877657249807705, -1.671971372180727, -0.2788017810010903, 0.22094417468367333, 4.33162015072758, 0.21102631103562178], 15)
      if chr(key) == '3':
        self.irpos.move_to_joint_position([0.3010880000000001, 0.5876247031931617, -1.671971372180727, -0.27889827803035894, 0.5205105370471284, 4.130648214437389, 0.21102631103562178], 15)
      
      if chr(key) == '4':
        self.irpos.move_to_joint_position([0.30109, 0.5876247031931617, -1.671971372180727, -0.17896565363985745, 0.5205443726415584, 4.1308936513634515, 0.21102631103562178], 15)

      if pos_current != old_pos:
        self.irpos.move_to_joint_position(pos_current, 1)
      
    
    
  def run(self):
    
    pos_current = self.irpos.get_joint_position()
    
    # initial pose
    pos_1 = [0.6344850000000001, -0.10578245759774751, -1.348892451559098, -0.3869163274131284, 0.00941591743894632, 3.575633512303192, 0.10135285361075765]
    # first closeup
    pos_2 = [-0.07029700000000001, -0.004993007211376222, -1.5484988567729139, 0.21336130858258034, 0.009187303373024314, 3.5770447746280474, 0.11040077791194643]
    # second closeup
    pos_3 = [-0.008298000000000002, -0.2057666263229138, -1.4495164345702594, 0.01332592670734925, 0.009660210745515041, 3.8752097336386715, 0.092685551125124]
    
    pos_4 = [-2.3000000000000003e-05, -0.10592164208240022, -1.2475318921395901, -0.08669530060982747, 0.6089224652366101, 3.3828019007273813, 0.7541083923782368]
    pos_5 = [-3.2000000000000005e-05, -0.10543449638611574, -1.6550936760194053, 0.3873075053980718, 1.3428021716105258, 2.6345056193195933, 1.507326340634153]
    
    # view from top
    pos_6 = [-0.08029300000000002, -0.3051940571094693, -1.3487689270267655, -0.18692129124737444, 0.009168917479207339, 4.075895326848458, 0.11045515245702557]



    pos1 = [0.21067300000000005, 0.5876247031931617, -1.671971372180727, -0.27887683340132824, 0.5204890924180978, 4.130689120591732, 0.21102631103562178]
    pos2 = [0.21067300000000005, 0.3877657249807705, -1.671971372180727, -0.2788017810010903, 0.22094417468367333, 4.33162015072758, 0.21102631103562178]
    pos3 = [0.3010880000000001, 0.5876247031931617, -1.671971372180727, -0.27889827803035894, 0.5205105370471284, 4.130648214437389, 0.21102631103562178]
    pos4 = [0.30109, 0.5876247031931617, -1.671971372180727, -0.17896565363985745, 0.5205443726415584, 4.1308936513634515, 0.21102631103562178]



    
    
    # maximum joint velocities
    max_vel = [ 0.05, 0.5, 0.5, 0.5, 0.5, 0.5, 0.25]
    
    
    diff = np.array(pos_current) - np.array(pos_1)
    print diff
    diff = np.absolute(diff)
    print diff
    diff = np.divide(diff, max_vel)
    print diff
    first_time = max(np.amax(diff), 1)
    print first_time

    # move to initial position
    print "Moving to initial position, as fast as possible, in", first_time, "seconds"
    self.irpos.move_to_joint_position(pos_1, first_time)
    

   # print "Waiting for transform..."
    #self.listener.waitForTransform('/world', '/t_c', rospy.Time.now(), rospy.Duration(5))
   # print "Got it!"
    
    self.running = False
    self.irpos.move_to_joint_position(pos1, 15)
    self.running = True
    self.irpos.move_to_joint_position(pos2, 5)
    self.irpos.move_to_joint_position(pos3, 5)
    self.irpos.move_to_joint_position(pos4, 5)
    self.running = False
    
    #self.irpos.move_to_joint_position(pos_1, 15)
    
    
    #fout = open('transform.txt', 'a')
    
    print "Saving", len(self.images), "images"
    for i in range(len(self.images)):
      fname = str(self.images[i].header.stamp.secs) + "_" + format(self.images[i].header.stamp.nsecs, '09') + ".png"
      print "Saving", fname
      
      #tr = self.listener.lookupTransform('/world', '/t_c', self.images[i].header.stamp - rospy.Duration(0.025))
      #fout.write(fname + "\t" + str(tr) + "\n")
      if i%20 == 0:
      	cv2.imwrite(fname, self.bridge.imgmsg_to_cv2(self.images[i], "bgr8"))
      #fout.write(fname + "\t" + str(self.trs[i]) + "\n")
      
      
    #fout.close()
    print "I'm done!"

  def synchro(self):
    self.irpos.move_to_synchro_position(15)

  def callback(self, data):
      
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      #tr = self.listener.lookupTransform('/world', '/t_c', data.header.stamp - rospy.Duration(0.025))

      if self.running:
        self.images.append(data)
        print len(self.images)

      cv2.imshow("Image window", cv_image)
      cv2.waitKey(3)
      
    except Exception, e:
      print e


def main(args):
  ic = image_converter()
  
  #ic.run()
  ic.step_move()
  #ic.synchro()
  
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

	
#pozycja w motorach nad pionkiem
#6.7151542970481835, 35.89583765991698, 8.898561191293089, 151.30852697484522, 70.34025951387547, 749.6374142216357
