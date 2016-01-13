#!/usr/bin/env python
import rospy

#pakiety do subskrybcji tf
import roslib
roslib.load_manifest('serwo')
import rospy
import math
import tf
from tf import transformations
import numpy as np
import geometry_msgs.msg

from std_msgs.msg import Float32
from serwo.msg import SerwoInfo
from irpos import *
import matrixOperations

#konwencja
#T_aB - pozycja ukladu a w ukladzie B
#w mojej notacji a stoi na dole, a B stoi na gorze
class serwoInfo:
  def __init__(self): 
    self.irpos = IRPOS("maciek", "Irp6p", 6, "irp6p_manager")
    self.listener = tf.TransformListener()

    #dane o obrazie otrzymane z topica
    self.T_gC_vector = []
    self.found = 1

    self.frequency = 500 #czestotliwosc wysylania sterowania dla robota

    self.error = [0,0,0,0,0,0] #uchyby dla 6 stopni swobody
    self.minError = [0.01,0.01,0.01,0.01,0.01,0.01] #minimalny uchyb przy ktorym zostanie wyliczona nowa predkosc

    self.newVel = [0,0,0,0,0,0] #wektro predkosci dla 6 stopni swobody
    self.newAcceleration = [0,0,0,0,0,0] #wektro przyspieszenia dla 6 stopni swobody

    self.p = [0,0.5,0,0,0,0] #wartosc p dla regulatora PID
    self.i = [0,0,0,0,0,0] #wartosc p dla regulatora PID
    self.d = [0,0,0,0,0,0] #wartosc p dla regulatora PID

    self.maxVel = [0.05,0.05,0.05,0,0,0]
    self.maxAcceleration = [25.0,25.0,25.0,0,0,0]
    self.new_pos_cartesian = [0.0, 0.0, 0.0]

    #kartezajanskie ograniczenia pozycji jako trojki xyz
    #w ukladzie zwiazanym z baza robota
    #przetestowac ograniczenia na prawdziwym robocie
    self.min_pos = [0.8, -0.3, 1.2] #xyz
    self.max_pos = [0.9, 0.3, 1.4] #xyz

    self.w_vector = [[0 for x in range(3)] for x in range(8)] 
    self.w_vector[0] = [self.max_pos[0], self.max_pos[1], self.min_pos[2]]
    self.w_vector[1] = [self.min_pos[0], self.max_pos[1], self.min_pos[2]]
    self.w_vector[2] = [self.min_pos[0], self.min_pos[1], self.min_pos[2]]
    self.w_vector[3] = [self.max_pos[0], self.min_pos[1], self.min_pos[2]]
    self.w_vector[4] = [self.max_pos[0], self.max_pos[1], self.max_pos[2]]
    self.w_vector[5] = [self.min_pos[0], self.max_pos[1], self.max_pos[2]]
    self.w_vector[6] = [self.min_pos[0], self.min_pos[1], self.max_pos[2]]
    self.w_vector[7] = [self.max_pos[0], self.min_pos[1], self.max_pos[2]]

  def limits_test(self):
  #przejazd robota po ograniczeniach kartezjanskich
    irpos.move_to_joint_position([0.0, -1.57079632679, -0.0, -0.0, 4.71238898038, 1.57079632679], 10.0) #pozycja pionowa
    for i in range(8):
      self.irpos.move_to_cartesian_pose(5.0,Pose(Point(self.w_vector[i][0], self.w_vector[i][1], self.w_vector[i][2]), Quaternion(1.0, 0.0, 0.0, 0.0)))

  def callback(self, msg):
    self.T_gC_vector = msg.matrix
    self.found = msg.found

  def calculateNewVel(self):
    for i in range(6):
      if self.error[i] > self.minError[i] or self.error[i] < -self.minError[i]:
        self.newVel[i] = self.error[i] * self.p[i]
      else:
        print 'error to small'
        self.newVel[i] = 0

  def calculateNewCartesianPose(self):
    current_pos_cartesian = self.irpos.get_cartesian_pose()
    self.new_pos_cartesian[0] = current_pos_cartesian.position.x + self.newVel[0]*(1/self.frequency)
    self.new_pos_cartesian[1] = current_pos_cartesian.position.y + self.newVel[1]*(1/self.frequency)
    self.new_pos_cartesian[2] = current_pos_cartesian.position.z + self.newVel[2]*(1/self.frequency)

  def calculateNewAcceleration(self):
    for i in range(3):
      self.newAcceleration[i] = self.newVel[i]*self.frequency

  def checkVelocityLimits(self):
    for i in range(3):
      if self.newVel[i] > self.maxVel[i]:
        self.newVel[i] = self.maxVel[i]
        print 'Przekroczono limit predkosci w osi ' + str(i) + " : " + str(self.newVel[i])
      if self.newVel[i] < -self.maxVel[i]:
	self.newVel[i] = -self.maxVel[i]
        print 'Przekroczono limit predkosci w osi ' + str(i) + " : " + str(self.newVel[i]) 
     
  def checkAccelerationLimits(self):
    for i in range(3):
      if not (self.newAcceleration[i] > -self.maxAcceleration[i] and self.newAcceleration[i] < self.maxAcceleration[i]):
        print 'Przekroczono limit przyspieszenia w osi ' + str(i) + ': ' + str(self.newAcceleration[i])
        self.newVel[0] = 0

  def checkCartesianLimits(self):
    for i in range(3):
      if not (self.new_pos_cartesian[i] > self.min_pos[i] and self.new_pos_cartesian[i] < self.max_pos[i]):
        print 'Przekroczony limit pozycji w osi ' + str(i) + ': ' + str(self.new_pos_cartesian[i])
        self.newVel[i] = 0

  def setZeroValues(self):
    self.newVel = [0,0,0,0,0,0] 
  
  def setNewVelocity(self):
    self.checkVelocityLimits()
    #self.checkAccelerationLimits()
    self.checkCartesianLimits()   

  def calculateAndSetNewValues(self):
    if self.found == 1:
      self.calculateNewVel()
      #self.calculateNewAcceleration()
      self.calculateNewCartesianPose()
      self.setNewVelocity()
      print 'a'
    else:
      self.setZeroValues()
      print 'b'

  def TestLimits(self):
    rate = rospy.Rate(self.frequency) #czestotliwosc pracy wezla
    self.irpos.set_tool_physical_params(10.8, Vector3(0.004, 0.0, 0.156)) #ustawienia do sterowania predkoscia

    current_vel = 0.000001
    for a in range(50):
      for x in range(12):
        current_vel = current_vel * 5
        self.newVel = [0,current_vel,0,0,0,0]
        print 'current_vel ='
        print current_vel
        self.calculateAndSetNewValues()
        self.irpos.start_force_controller(Inertia(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), ReciprocalDamping(Vector3(0.0025, 0.0025, 0.0025), Vector3(0.0, 0.0, 0.0)), Wrench(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), Twist(Vector3(self.newVel[0], self.newVel[1], self.newVel[2]), Vector3(self.newVel[3], self.newVel[4], self.newVel[5]))) #sterowanie predkoscia
        rate.sleep()

    self.irpos.stop_force_controller()

  def regulatroDecomposition(self):
    self.error[0] = self.T_gC[1,3] #uchyb w osi x kamery
    self.error[1] = self.T_gC[0,3] #uchyb w osi y kamery
    self.error[2] = self.T_gC[2,3] #uchyb w osi z kamery
    #TODO dekopozycja katow os-kat
      
  def run(self):
    rate = rospy.Rate(self.frequency) #czestotliwosc pracy wezla
    self.irpos.set_tool_physical_params(10.8, Vector3(0.004, 0.0, 0.156)) #ustawienia do sterowania predkoscia

    pub = rospy.Publisher('uchyb', Float32, queue_size=10) #publikacja uchybu do celu rysowania wykresy
    rospy.Subscriber("object_seen_by_camera", SerwoInfo, self.callback) #subskrybcja T_gC_vector
 
    while not rospy.is_shutdown():
        if not not self.T_gC_vector:
          self.T_gC = matrixOperations.translation_from_vector(self.T_gC_vector) #przeksztalcenie wektora na macierz
	  self.regulatroDecomposition()
	  self.calculateAndSetNewValues()
	  self.irpos.start_force_controller(Inertia(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), ReciprocalDamping(Vector3(0.0025, 0.0025, 0.0025), Vector3(0.0, 0.0, 0.0)), Wrench(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), Twist(Vector3(self.newVel[0], self.newVel[1], self.newVel[2]), Vector3(self.newVel[3], self.newVel[4], self.newVel[5]))) #sterowanie predkoscia

        rate.sleep()
        pub.publish(self.error[1]) #publikacja uchybu
    self.irpos.stop_force_controller()   
def main():
  si = serwoInfo()
  si.run()

if __name__ == '__main__':
  main()

