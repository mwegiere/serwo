#!/usr/bin/env python
import rospy
from irpos import *
from std_msgs.msg import *


if __name__ == '__main__':
	irpos = IRPOS("maciek", "Irp6p", 6, "irp6p_manager")
	rate = rospy.Rate(500)
	irpos.set_tool_physical_params(10.8, Vector3(0.004, 0.0, 0.156)) #ustawienia do sterowania predkoscia
    	while not rospy.is_shutdown():
		irpos.start_force_controller(Inertia(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), ReciprocalDamping(Vector3(0.0025, 0.0025, 0.0025), Vector3(0.0, 0.0, 0.0)), Wrench(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), Twist(Vector3(0, 0.01, 0), Vector3(0, 0, 0))) 

        rate.sleep()

#	def set_force_controller_goal(self, inertia, reciprocaldamping, wrench, twist):
#		forceControlGoal = ForceControl()
#		forceControlGoal.inertia = inertia
#		forceControlGoal.reciprocaldamping = reciprocaldamping
#		forceControlGoal.wrench = wrench
#		forceControlGoal.twist = twist
  
#		self.fcl_param_publisher.publish(forceControlGoal)
 
#		tg_goal = ToolGravityParam()
#		tg_goal.weight = self.tool_weight
#		tg_goal.mass_center = self.tool_mass_center
# 
#		self.tg_param_publisher.publish(tg_goal)

#self.fcl_param_publisher = rospy.Publisher('/'+robotNameLower+'_arm/fcl_param', ForceControl, queue_size=0)
#self.tg_param_publisher = rospy.Publisher('/'+robotNameLower+'_arm/tg_param', ToolGravityParam, queue_size=0)
