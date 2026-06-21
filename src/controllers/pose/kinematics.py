#!/usr/bin/pythetaon2

## UR5/UR10 Inverse Kinematics - Ryan Keating Johns Hopkins University


# ***** lib
import numpy as np

import cmath
import math
from numpy import cos as cos
from numpy import cos as cos
from numpy import sin as sin
from numpy import arctan2 as atan2
from numpy import arccos as acos
from numpy import arcsin as asin
from numpy import sqrt as sqrt
from numpy import pi as pi


class Kinematics:
  def __init__(
    self,
    robot,
    ):
    self._pybullet_client = robot.pybullet_client
    self._robot = robot

    self._a = np.array([0 ,-0.612 ,-0.5723 ,0 ,0 ,0])
    self._d = np.array([0.1273, 0, 0, 0.163941, 0.1157, 0.0922])
    self._alpha = np.array([pi/2, 0, 0, pi/2, -pi/2, 0 ])

  def _get_Rzt(self, n, theta, c):
    return np.array([[cos(theta[n-1,c]), -sin(theta[n-1,c]),  0 ,0],
                      [sin(theta[n-1,c]),  cos(theta[n-1,c]), 0, 0],
                      [0,                  0,                 1, 0],
                      [0,                  0,                 0, 1]])
    

  def _get_Rxa(self, n, theta, c):
    return np.array([[1,                  0,                      0,                       0],
                     [0,                  cos(self._alpha[n-1]), -sin(self._alpha[n-1]),   0],
                     [0,                  sin(self._alpha[n-1]),  cos(self._alpha[n-1]),   0],
                     [0,                  0,                      0,                       1]])
    
  def _get_AH(self, n, theta, c):
    T_a = np.eye(4)
    T_a[0, 3] = self._a[n-1]
    T_d = np.eye(4)
    T_d[2, 3] = self._d[n-1]

    Rzt = self._get_Rzt(n, theta, c)
    Rxa = self._get_Rxa(n, theta, c)

    A_i = T_d @ Rzt @ T_a @ Rxa
  
    return A_i
  
  def _compute_jacobian(self, link_id):
        """Computes the Jacobian matrix for the given link.
        Args:
          link_id: The link id as returned from loadURDF.
        Returns:
          The 3 x N transposed Jacobian matrix. where N is the total DoFs of the
          robot. For a _quadruped, the first 6 columns of the matrix corresponds to
          the CoM translation and rotation. The columns corresponds to a leg can be
          extracted with indices [6 + leg_id * 3: 6 + leg_id * 3 + 3].
        """
        joint_angles = [state[0] for state in self.self._robot.GetJointStates]
        zero = [0] * len(joint_angles)
        jv, _ = self._pybullet_client.calculateJacobian(self._robot.get_robot_id,
                                                        link_id,
                                                        (0, 0, 0),
                                                        joint_angles,
                                                        zero,
                                                        zero)
        jacobian = np.array(jv)
        assert jacobian.shape[0] == 3

        return jacobian
  
  def forwardKinematics(self, theta, c):
      T = np.eye(4)
      for n in range(1, 7):
        T = T @ self._get_AH(n, theta, c)
          
      return T
  
  def inverseKinematics(self, coord):
    theta = np.zeros((6, 8))
    P_05 = (coord @ np.array([0, 0, -self._d[5], 1]) - np.array([0, 0, 0, 1]))

    # **** theta1 ****
    psi = atan2(P_05[1], P_05[0])
    phi = acos(self._d[3] /sqrt(P_05[1]**2 + P_05[0]**2))
    
    theta[0, 0:4] = pi/2 + psi + phi
    theta[0, 4:8] = pi/2 + psi - phi
    theta = theta.real
    
    # **** theta5 ****
    cl = [0, 4]   # wrist up or down
    for i in range(0, len(cl)):
      c = cl[i]
      T_10 = np.linalg.inv(self._get_AH(1, theta, c))
      T_16 = T_10 @ coord
      theta[4, c:c+2] = +acos((T_16[2, 3] - self._d[3])/self._d[5])
      theta[4, c+2:c+4] = - acos((T_16[2, 3]-self._d[3])/self._d[5])

    theta = theta.real

    # **** theta6 ****
    cl = [0, 2, 4, 6]
    for i in range(0, len(cl)):
      c = cl[i]
      T_10 = np.linalg.inv(self._get_AH(1, theta, c))
      T_16 = np.linalg.inv(T_10 @ coord)
      theta[5, c:c+2] = atan2((-T_16[1, 2]/sin(theta[4, c])), (T_16[0, 2]/sin(theta[4, c])))
    
    theta = theta.real

    # **** theta3 ****
    cl = [0, 2, 4, 6]
    for i in range(0,len(cl)):
      c = cl[i]
      T_10 = np.linalg.inv(self._get_AH(1, theta, c))
      T_65 = self._get_AH(6, theta, c)
      T_54 = self._get_AH(5, theta, c)
      T_14 = (T_10 @ coord) @ np.linalg.inv(T_54 @ T_65)
      P_13 = T_14 @ np.array([0, -self._d[3], 0, 1]) - np.array([0, 0, 0, 1])
      t3 = cmath.acos((np.linalg.norm(P_13)**2 - self._a[1]**2 - self._a[2]**2 )/(2 * self._a[1] * self._a[2])) 
      theta[2, c] = t3.real
      theta[2, c+1] = -t3.real

    # **** theta2 and theta4 ****

    cl = [0, 1, 2, 3, 4, 5, 6, 7]
    for i in range(0, len(cl)):
      c = cl[i]
      T_10 = np.linalg.inv(self._get_AH(1, theta, c))
      T_65 = np.linalg.inv(self._get_AH(6, theta, c))
      T_54 = np.linalg.inv(self._get_AH(5, theta, c))
      T_14 = (T_10 @ coord) @ T_65 @ T_54
      P_13 = T_14 @ np.array([0, -self._d[3], 0, 1]).T - np.array([0, 0, 0, 1]).T
      

      theta[1, c] = -atan2(P_13[1], -P_13[0]) + asin(self._a[2] * sin(theta[2, c])/np.linalg.norm(P_13))
      T_32 = np.linalg.inv(self._get_AH(3, theta, c))
      T_21 = np.linalg.inv(self._get_AH(2, theta, c))
      T_34 = T_32 @ T_21 @ T_14


      theta[3, c] = atan2(T_34[1, 0], T_34[0, 0])
    theta = theta.real

    return theta
  
  def solve_IK(self, link_position, link_orientation, link_id):
      lower_limits, upper_limits, joint_ranges = self._robot.get_joint_limits
      joint_angles = self._pybullet_client.calculateInverseKinematics(
          self._robot.get_robot_id,
          link_id,
          link_position,
          link_orientation,
          lowerLimits=lower_limits,
          upperLimits=upper_limits,
          jointRanges=joint_ranges,
          maxNumIterations=100,
          residualThreshold=1e-5
      )
      return list(joint_angles[:6]) 


