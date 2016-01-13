#!/usr/bin/env python
import rospy
import numpy as np
from tf import transformations

#konwencja
#T_AB - macierz przeksztacenia jednorodnego pomiedzy ukladem B, a ukladem A, czyli pozycja ukladu A w ukladzie B
#w mojej notacji A stoi na dole, a B stoi na gorze



def translation_from_vector(R):
  return np.array([ [R[0], R[1], R[2], R[3]],
                    [R[4], R[5], R[6], R[7]],
                    [R[8], R[9], R[10], R[11]],
                    [R[12], R[13], R[14], R[15]]]);



def vector_from_translation(R):
  return [ R[0,0], R[0,1], R[0,2], R[0,3], R[1,0], R[1,1], R[1,2], R[1,3], R[2,0], R[2,1], R[2,2], R[2,3], R[3,0], R[3,1], R[3,2], R[3,3]];

#def euler_matrix_from_quaternion(rotQuaternion, trans):

#  euler_from_quaternion = transformations.euler_from_quaternion(rotQuaternion)
#  euler_matrix = transformations.euler_matrix(euler_from_quaternion[0], euler_from_quaternion[1], euler_from_quaternion[2])
#  return np.array([ [euler_matrix[0,0], euler_matrix[0,1], euler_matrix[0,2], trans[0] ],
#                    [euler_matrix[1,0], euler_matrix[1,1], euler_matrix[1,2], trans[1] ],
#                    [euler_matrix[2,0], euler_matrix[2,1], euler_matrix[2,2], trans[2] ],
#                    [                0,                 0,                 0,        1 ] ]);

def euler_matrix_from_quaternion(rotQuaternion, trans):
  euler_matrix = transformations.quaternion_matrix(rotQuaternion)
  euler_matrix[0,3] = trans[0]
  euler_matrix[1,3] = trans[1]
  euler_matrix[2,3] = trans[2]
  return euler_matrix            

#dekompozycja regulatora

#regulator proporcjonalny

