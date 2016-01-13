#!/usr/bin/env python
import time
import rospy
from irpos import *
from std_msgs.msg import *

if __name__ == '__main__':
	irpos = IRPOS("startConman", "Irp6p", 6, "irp6p_manager")
	conmanSwitch = rospy.ServiceProxy('/irp6p_manager/switch_controller', SwitchController)
	conmanSwitch(['Irp6pmForceTransformation'], [], True)
	time.sleep(0.05)
	conmanSwitch(['Irp6pmForceControlLaw'], [], True)

	
