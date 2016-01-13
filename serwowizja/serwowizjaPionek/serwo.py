#!/usr/bin/env python
import rospy
from irpos import *
from std_msgs.msg import *
import threading

x_px = 0
y_px = 0
wsp = 0.00027473
z = 1.10342843843
#dla obrazu pelnego
#wartosc_punkt_odniesienia_y = 649
#dla obrazu ucietego
wartosc_punkt_odniesienia_y = 300

xLock = threading.Lock()
yLock = threading.Lock()

def pozycja_startowa():
	irpos.move_to_motor_position([-14.845596084538569, 14.828317324943825, 2.7064820710676067, 147.2464476737536, 69.27840119696212, 1217.3294541542018], 10.0)

def wyswietl_aktualna_pozycje():
	print "joint position"
	print irpos.get_joint_position()
	print "motor position"
	print irpos.get_motor_position()
	print "cartesian position"
	print irpos.get_cartesian_pose()
	print "tfg joint  position"
	print irpos.get_tfg_joint_position()
	print "tfg motor position"
	print irpos.get_tfg_motor_position()

def zlap_pionek():
	#rozszerz chytak
	irpos.tfg_to_joint_position(0.07, 5.0)
	#idz na dol 3.0 N, 30 cm w dol
	irpos.move_rel_to_cartesian_pose_with_contact(50.0, Pose(Point(0.0, 0.0, 0.3), Quaternion(0.0, 0.0, 0.0, 1.0)), Wrench(Vector3(0.0,0.0,3.0),Vector3(0.0,0.0,0.0)))
	#lekko w gore
	irpos.move_rel_to_cartesian_pose(30.0, Pose(Point(0.0, 0.0, -0.005), Quaternion(0.0, 0.0, 0.0, 1.0)))
	#lap
	irpos.tfg_to_joint_position(0.063, 5.0)

def sterowanie_silowe():
	irpos.set_tool_physical_params(10.8, Vector3(0.004, 0.0, 0.156))
	irpos.start_force_controller(Inertia(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), ReciprocalDamping(Vector3(0.0025, 0.0025, 0.0025), Vector3(0.0, 0.0, 0.0)), Wrench(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)))

	time.sleep(20.0)
	irpos.stop_force_controller()

def sterowanie_silowe2(predkosc_y):
	irpos.set_tool_physical_params(10.8, Vector3(0.004, 0.0, 0.156))
	irpos.start_force_controller(Inertia(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), ReciprocalDamping(Vector3(0.0025, 0.0025, 0.0025), Vector3(0.0, 0.0, 0.0)), Wrench(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), Twist(Vector3(0.0, predkosc_y, 0.0), Vector3(0.0, 0.0, 0.0)))

	time.sleep(10.0)
	irpos.stop_force_controller()

def odczytaj_pozycje_y_px(data):
	xLock.acquire()
	global y_px
	y_px = data.data
	xLock.release()

def oblicz_przesuniecie_y_cm(punkt_odniesienia_y, y_px):
	wsp = (1.10342843843-0.987109990995)*z - 0.1143
#(1.10342843843-0.987109990995)*0.987109990995-0.0001476
	print "y_px"
	print y_px
	print "punkt_odniesienia_y"
	print punkt_odniesienia_y
	#wynik = (punkt_odniesienia_y - y_px)*wsp*0.8
	wynik = (punkt_odniesienia_y - y_px)*0.00027473
	#0.0001476 dla pozycjo u dolu kartezjansko: z: 0.987109990995
	#gorna pozycja z: 1.10342843843 wsp = 0.00027473
	return wynik

def przesun_nad_pionek(y_cm,czas):
	#if (y_cm > - 0.01 and y_cm <0.01 and z > 0.987109990995):
	#	z = z - 0.01
	#	move_rel_to_cartesian_pose(czas, Pose(Point(0.0, y_cm, -0.01), Quaternion(0.0, 0.0, 0.0, 1.0)))
	#if ((y_cm < - 0.01 or y_cm >0.01) and z < 1.10342843843):
	#	z = z + 0.01
	#	move_rel_to_cartesian_pose(czas, Pose(Point(0.0, y_cm, 0.01), Quaternion(0.0, 0.0, 0.0, 1.0)))
	#else:
	move_rel_to_cartesian_pose(czas, Pose(Point(0.0, y_cm, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)))
	
def move_rel_to_cartesian_pose(time_from_start, rel_pose):
		print irpos.BCOLOR+"[IRPOS] Move relative to cartesian trajectory"+irpos.ENDC
				
		#irpos.conmanSwitch([irpos.robot_name+'mPoseInt'], [], True)

		actual_pose = irpos.get_cartesian_pose()

		# Transform poses to frames.
		actualFrame = pm.fromMsg(actual_pose)
		
		relativeFrame = pm.fromMsg(rel_pose)
		
		desiredFrame = actualFrame * relativeFrame
		
		pose = pm.toMsg(desiredFrame)

		cartesianGoal = CartesianTrajectoryGoal()
		cartesianGoal.trajectory.points.append(CartesianTrajectoryPoint(rospy.Duration(time_from_start), pose, Twist()))
		cartesianGoal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.05)
		
		irpos.pose_client.send_goal(cartesianGoal)
		#irpos.pose_client.wait_for_result()

		#result = irpos.pose_client.get_result()
		#code = irpos.cartesian_error_code_to_string(result.error_code)
		#print irpos.BCOLOR+"[IRPOS] Result: "+str(code)+irpos.ENDC

		#irpos.conmanSwitch([], [irpos.robot_name+'mPoseInt'], True)	
def listener_y():
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("center_y", int, odczytaj_pozycje_y_px)

def serwo():
	rospy.Subscriber('/center_x', Int32, odczytaj_pozycje_y_px)
	rospy.sleep(1.)
	licznik = 0
	irpos.conmanSwitch([irpos.robot_name+'mPoseInt'], [], True)
	while(licznik<900):

		y_cm = oblicz_przesuniecie_y_cm(wartosc_punkt_odniesienia_y, y_px)
		print y_cm
		czas = abs(y_cm * 30)
		if ((y_cm < 0.3 and y_cm > -0.3) and not (y_cm < 0.001 and y_cm > -0.001) and y_px !=0):
			print "ok"
			przesun_nad_pionek(-y_cm, czas)
		else:
			print "zle"
		licznik = licznik +1;
		rospy.sleep(0.1)
	
	irpos.conmanSwitch([], [irpos.robot_name+'mPoseInt'], True)


if __name__ == '__main__':
	#irpos = IRPOS("maciek", "Irp6p", 6, "irp6p_manager")
	#irpos = IRPOS("maciek", "Irp6p", 6)
	irpos = IRPOS("maciek", "Irp6p", 6, "irp6ot_manager")
	pozycja_startowa()
	#sterowanie_silowe2(0.01)
	#sterowanie_silowe2(-0.01)
	#wyswietl_aktualna_pozycje()
	#serwo()
	#sterowanie_silowe()
