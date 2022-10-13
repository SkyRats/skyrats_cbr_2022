import cv2
import rclpy
import numpy as np
from pickle import FALSE
import time

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
        self.bases_visited=[]
        self.altura = 3
    #def precision_land(self):   
  
    #def findDisplay(self, mask_tube): 
    
    #def readDisplay(self, mask_tube):
 
    def trajectory(self):

        rclpy.spin_once(self.mav2) 
        self.mav2.takeoff(self.altura)
        current_x = self.mav2.drone_pose.pose.position.x
        current_y = self.mav2.drone_pose.pose.position.y
        dist_menor=8*(2**(1/2))
        qtdade_bases_visited = 0


        self.bases_not_visited.append([0.25, -6.25, 1])
        self.bases_not_visited.append([2.75, 0.25, 1.5])
        self.bases_not_visited.append([4, -3.5, 0])
        self.bases_not_visited.append([6, -2, 0])
        self.bases_not_visited.append([6, -6.5, 0])

        while(qtdade_bases_visited!=5):
            rclpy.spin_once(self.mav2) 
            current_x = self.mav2.drone_pose.pose.position.x
            current_y = self.mav2.drone_pose.pose.position.y
            for i in range(0,len(self.bases_not_visited)):
                
                if self.bases_not_visited[i] not in self.bases_visited:
                    print("entrei")
                  
                    X=self.bases_not_visited[i][0]
                    Y=self.bases_not_visited[i][1]


                    dist=np.sqrt((X- current_x)**2 + (Y - current_y)**2)

                    if dist<dist_menor:
                        dist_menor=dist
                        i_oficial=i
            dist_menor = 8*(2**(1/2))
            self.mav2.go_to_local(current_x, current_y, 3)
            self.mav2.go_to_local(self.bases_not_visited[i_oficial][0],self.bases_not_visited[i_oficial][1], 3)
            self.mav2.go_to_local(self.bases_not_visited[i_oficial][0],self.bases_not_visited[i_oficial][1], self.bases_not_visited[i_oficial][2] + 0.5)
            self.bases_visited.append(self.bases_not_visited[i_oficial]) 
            # print(self.bases_visited)       
            qtdade_bases_visited += 1
        rclpy.spin_once(self.mav2) 
        current_x = self.mav2.drone_pose.pose.position.x
        current_y = self.mav2.drone_pose.pose.position.y    
        self.mav2.go_to_local(current_x, current_y, 3)
        self.mav2.go_to_local(0, 0, 3)
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
