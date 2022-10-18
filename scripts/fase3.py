#!/usr/bin/env python3
import cv2
import rospy
import numpy as np
import time
import math
from MAV_ardupilot import MAV2
from plaquinha_classe import Buzzer
from sensorDetector import sensorDetection

TOL = 0.5 #tolerancia de erro das bases 
INIT_HEIGHT = 0.5

class fase3:
    def __init__(self, mav):
        self.mav = mav
        self.bases_not_visited=[]
        
        # altura do voo em relação ao tamanho incial da base costeira
        self.altitude = 1.5 - INIT_HEIGHT

        # velocidade de cruzeiro
        self.vel_cruzeiro = 0.5

 
    def trajectory(self):

        base1=(-4.05, -0.55, 1)
        base2=(-6, 2, 0)
        base3=(-4, 5, 0)
        base4=(-3.58, 7.02, 1)
        base5=(-2, 6, 0)

        self.bases_not_visited.append(base1)
        self.bases_not_visited.append(base2)
        self.bases_not_visited.append(base3)
        self.bases_not_visited.append(base4)
        self.bases_not_visited.append(base5)     
        qtdade_bases_visited = 0
        self.mav.takeoff(self.altitude) 
        rospy.sleep(7)
        while(qtdade_bases_visited!=5):

            self.mav.go_to_local(self.bases_not_visited[qtdade_bases_visited][0],self.bases_not_visited[qtdade_bases_visited][1], self.altitude,  yaw=math.pi/2, sleep_time=10)
            self.mav.go_to_local(self.bases_not_visited[qtdade_bases_visited][0],self.bases_not_visited[qtdade_bases_visited][1], self.bases_not_visited[qtdade_bases_visited][2] + 0.5,  yaw=math.pi/2, sleep_time=10)     
            rospy.sleep(3)
            self.mav.go_to_local(self.bases_not_visited[qtdade_bases_visited][0],self.bases_not_visited[qtdade_bases_visited][1], self.altitude,  yaw=math.pi/2, sleep_time=10)
            qtdade_bases_visited += 1
   
        self.mav.go_to_local(0, 0, self.altitude, yaw=math.pi/2, sleep_time=10)
        self.mav.land()
        
        # current_x = 0
        # current_y = 0
        # while(qtdade_bases_visited!=5):
        #     for i in range(0,len(self.bases_not_visited)):
        #         if self.bases_not_visited[i] not in self.bases_visited:
        #             print("entrei")
        #             X=self.bases_not_visited[i][0]
        #             Y=self.bases_not_visited[i][1]
        #             dist=np.sqrt((X- current_x)**2 + (Y - current_y)**2)
        #             if dist<dist_menor:
        #                 dist_menor=dist
        #                 i_oficial=i
        #     dist_menor = 8*(2**(1/2))
        #     self.mav2.go_to_local(current_x, current_y, 3,  yaw=math.pi/2, sleep_time=10)
        #     self.mav2.go_to_local(self.bases_not_visited[i_oficial][0],self.bases_not_visited[i_oficial][1], 3, yaw=math.pi/2, sleep_time=10)
        #     self.mav2.go_to_local(self.bases_not_visited[i_oficial][0],self.bases_not_visited[i_oficial][1], self.bases_not_visited[i_oficial][2] + 0.5, yaw=math.pi/2, sleep_time=10)
        #     current_x = self.bases_not_visited[i_oficial][0]
        #     current_y = self.bases_not_visited[i_oficial][1]
        #     self.bases_visited.append(self.bases_not_visited[i_oficial]) 
        #     qtdade_bases_visited += 1 
        # self.mav2.go_to_local(current_x, current_y, 3, yaw=math.pi/2, sleep_time=10)
        # self.mav2.go_to_local(0, 0, 3, yaw=math.pi/2, sleep_time=10)
        # self.mav2.land()            

if __name__ == "__main__":
    rospy.init_node('fase2')
    mav = MAV2()
    missao = fase3(mav)
    missao.trajectory()
