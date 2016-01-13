#!/usr/bin/env python
import rospy
from irpos import *
from std_msgs.msg import *
import threading

x_px = 0
y_px = 0
wsp = 0.00027473
z = 1.10342843843
wartosc_punkt_odniesienia_y = 144
#wartosc_punkt_odniesienia_y = 111

xLock = threading.Lock()
yLock = threading.Lock()

def pozycja_startowa():
	irpos.move_to_motor_position([-14.845596084538569, 14.828317324943825, 2.7064820710676067, 147.2464476737536, 69.27840119696212, 1217.3294541542018], 10.0)

def pozycja_startowa_wysoka():
	irpos.move_to_motor_position([-24.787166036823468, 23.865108592994865, -30.200130178958684, 148.34443430618325, 75.49404226208952, 353.6459434219494], 10.0)

def sterowanie_silowe():
	irpos.set_tool_physical_params(10.8, Vector3(0.004, 0.0, 0.156))
	irpos.start_force_controller(Inertia(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), ReciprocalDamping(Vector3(0.0025, 0.0025, 0.0025), Vector3(0.0, 0.0, 0.0)), Wrench(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)))

	time.sleep(60.0)
	irpos.stop_force_controller()

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

def odczytaj_pozycje_y_px(data):
	xLock.acquire()
	global y_px
	y_px = data.data
	xLock.release()

def oblicz_przesuniecie_y_cm(punkt_odniesienia_y, y_px):
	#wynik = (punkt_odniesienia_y - y_px)*0.00052632*0.8
	wynik = (punkt_odniesienia_y - y_px)*0.00052632*0.8
	return wynik

def listener_y():
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("center_y", int, odczytaj_pozycje_y_px)

def sterowanie_predkoscia(predkosc_y):
	irpos.set_tool_physical_params(10.8, Vector3(0.004, 0.0, 0.156))
	irpos.start_force_controller(Inertia(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), ReciprocalDamping(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), Wrench(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), Twist(Vector3(0.0, predkosc_y, 0.0), Vector3(0.0, 0.0, 0.0)))

	#time.sleep(10.0)
	#irpos.stop_force_controller()

def serwo():
	rospy.Subscriber('/center_x', Int32, odczytaj_pozycje_y_px)
	rospy.sleep(1.)
	licznik = 0
	while(licznik<500):
		y_cm = oblicz_przesuniecie_y_cm(wartosc_punkt_odniesienia_y, y_px)
		print "y_cm = " 
		print y_cm
		print y_px
		if ((y_cm < 0.06 and y_cm > -0.06) and not (y_cm < 0.001 and y_cm > -0.001) and y_px !=0):
			print "ok"
			sterowanie_predkoscia(-y_cm)
			rospy.sleep(1.0)
			#irpos.stop_force_controller()
			licznik = licznik +1;
		else:
			print "zle"
		
	irpos.stop_force_controller()
if __name__ == '__main__':
	#irpos = IRPOS("maciek", "Irp6p", 6, "irp6p_manager")
	#irpos = IRPOS("maciek", "Irp6ot", 7, "irp6ot_manager")
	irpos = IRPOS("maciek", "Irp6p", 6)
	pozycja_startowa()
	#pozycja_startowa_wysoka()
	#sterowanie_silowe()
	#wyswietl_aktualna_pozycje()
	serwo()
	
