#!/usr/bin/env python3
import cv2
import rclpy
import numpy as np
from pickle import FALSE
import time
sys.path.insert(0,'/home/' + os.environ["USER"] + '/skyrats_ws2/src/mavbase2')
from MAV2 import MAV2

from tf_transformations import quaternion_from_euler, euler_from_quaternion

TOL = 0.5 #tolerancia de erro das bases 
CAM_FOV = 1.64061 #94 graus

class base:
    def __init__(self, name, mav2):
        self.mav = mav2
        self._name = name
        self._cordenates = [self.mav.drone_pose.pose.position.x, self.mav.drone_pose.pose.position.y, self.mav.drone_pose.pose.position.z]

class fase1:
    def __init__(self, mav2):
        self.mav2 = mav2
        

    #def precision_land(self):   
  
    def findDisplay(self, mask_tube): 
    
    def readDisplay(self, mask_tube):
 
    def trajectory(self):


if __name__ == "__main__":
    rclpy.init()
    mav = MAV2()
    missao = fase1(mav)
    missao.trajectory()
    
