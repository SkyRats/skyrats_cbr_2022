import cv2
import rclpy
import numpy as np
from pickle import FALSE
import time
import math
import sys
import os
sys.path.insert(0,'/home/' + os.environ["USER"] + '/skyrats_ws2/src/mavbase2')
from MAV2 import MAV2

from tf_transformations import quaternion_from_euler, euler_from_quaternion

TOL = 0.5 #tolerancia de erro das bases 
CAM_FOV = 1.64061 #94 graus

class fase3:
    def __init__(self, mav2):
        self.mav2 = mav2
        self.bases_not_visited=[]
        self.altura = 3
    #def precision_land(self):   
  
    #def findDisplay(self, mask_tube): 
    
    #def readDisplay(self, mask_tube):
 
    def trajectory(self):

        base1=(0.25, -6.25, 1)
        base2=(2.75, 0.25, 1.5)
        base3=(4, -3.5, 0)
        base4=(6, -2, 0)
        base5=(6, -6.5, 0)

        self.bases_not_visited.append(base1)
        self.bases_not_visited.append(base2)
        self.bases_not_visited.append(base3)
        self.bases_not_visited.append(base4)
        self.bases_not_visited.append(base5)     
        qtdade_bases_visited = 0
        self.mav2.takeoff(self.altura) 
        while(qtdade_bases_visited!=5):

            self.mav2.go_to_local(self.bases_not_visited[qtdade_bases_visited][0],self.bases_not_visited[qtdade_bases_visited][1], 3,  yaw=math.pi/2, sleep_time=10)
            self.mav2.go_to_local(self.bases_not_visited[qtdade_bases_visited][0],self.bases_not_visited[qtdade_bases_visited][1], self.bases_not_visited[qtdade_bases_visited][2] + 0.5,   yaw=math.pi/2, sleep_time=10)     
            qtdade_bases_visited += 1
   
        self.mav.go_to_local(0, 0, self.altura + 0.5, yaw=math.pi/2, sleep_time=10)
        self.mav2.land()
            

if __name__ == "__main__":
    rclpy.init()
    mav = MAV2()
    missao = fase3(mav)
    rclpy.spin_once(mav)
    current_x = mav.drone_pose.pose.position.x
    current_y = mav.drone_pose.pose.position.y
    if current_x >= TOL or current_y >= TOL:
        mav.go_to_local(0, 0, 3)
    else:
        # missao.teste()
        missao.trajectory()
