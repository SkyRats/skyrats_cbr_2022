#!/usr/bin/env python3
from baseDetector import CrossDetection
import cv2
import rospy
import numpy as np
import time
import math
from MAV_ardupilot import MAV2


TOL = 0.5 #tolerancia de erro das bases 
INIT_HEIGHT = 0.5

# [self.mav.drone_pose.pose.position.x, self.mav.drone_pose.pose.position.y, self.mav.drone_pose.pose.position.z]

class fase1:
    def __init__(self, mav):

        self.bases_visited = 0
        self.bases_stored = []   

        self.mav = mav
        
        # velocidade de cruzeiro
        self.vel_cruzeiro = 0.2

        # altura do voo em relação ao tamanho incial da base costeira
        self.altitude = 1.5 - INIT_HEIGHT


    def land_known_base(self):
        self.mav.land()
        time.sleep(10)
        self.mav.takeoff(0.5) 
        rospy.sleep(7)


    def trajectory(self):
        self.mav.takeoff(self.altitude)
        rospy.sleep(7)
        self.mav.change_auto_speed(self.vel_cruzeiro)
        self.bases_stored.append(["0", 0, 0, INIT_HEIGHT])
        self.bases_visited += 1 
        self.bases_stored.append(["A", -4.05, -0.55, 1.0])
        self.bases_visited += 1 
        self.bases_stored.append(["B", -3.58, 7.02, 1.0])
        self.bases_visited += 1 

        self.mav.go_to_local(0, 0, self.altitude, yaw=math.pi/2, sleep_time=10)
        ######### Base aerea 1 ################
        self.mav.go_to_local(self.bases_stored[1][1], self.bases_stored[1][2], self.altitude, yaw=math.pi/2, sleep_time=10)
        self.land_known_base()
        self.mav.go_to_local(self.bases_stored[1][1], self.bases_stored[1][2], self.altitude, yaw=math.pi/2, sleep_time=10)
        ######### fim base aerea 1 ################

        ######### Base area 2 ################
        self.mav.go_to_local(self.bases_stored[2][1], self.bases_stored[2][2], self.altitude, yaw=math.pi/2, sleep_time=10)
        self.land_known_base()
        self.mav.go_to_local(self.bases_stored[2][1], self.bases_stored[2][2], self.altitude, yaw=math.pi/2, sleep_time=10)
        ######### fim base aerea 1 ################ 
        self.mav.go_to_local(0, 0, self.altitude, yaw=math.pi/2, sleep_time=10)
        self.mav.land()

if __name__ == "__main__":
    rospy.init_node('fase1')
    mav = MAV2()
    missao = fase1(mav)
    missao.trajectory()
    



    
