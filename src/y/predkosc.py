#!/usr/bin/env python
import rospy
from irpos import *
from std_msgs.msg import *

def pozycja_pionowa():
	irpos.move_to_joint_position([0.0, -1.57079632679, -0.0, -0.0, 4.71238898038, 1.57079632679], 10.0)
        print "Pozycja startowa ustawiona"

def ustawianie_narzedzia(z):
        print "Start ustawiania narzedzia"
	irpos.set_tool_geometry_params(Pose(Point(0.0, 0.0, z), Quaternion(0.0, 0.0, 0.0, 1.0)))
        print "Koniec ustawiania narzedzia"

def obracanie_chwytaka():
	for i in range(70):
		#irpos.move_rel_to_cartesian_pose(40.0,Pose(Point(0.0, 0.0, 0.0), Quaternion(0.60876, 0.0, 0.0, -0.79335)))#-75 st.
		irpos.move_rel_to_cartesian_pose(1.0,Pose(Point(0.0, 0.0, 0.0), Quaternion(-0.00872654, 0.0, 0.0, 0.99996192)))

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

if __name__ == '__main__':
	irpos = IRPOS("maciek", "Irp6p", 6, "irp6p_manager")
        #0.ustawienie magicznej liczby
        #1. ustawienie robota w pozycji pionowej
        pozycja_pionowa()
        #2. ustawienie narzedzia o dlugosci - odleglosc przeciecia osi nad chwytakiem do tasmociagu 
	#ustawianie_narzedzia(0.5)
	#3. ustawienie robota pod katem -45 st.
	#wyswietl_aktualna_pozycje()
	#obracanie_chwytaka()
	#wyswietl_aktualna_pozycje()

	
