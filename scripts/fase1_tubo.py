#!/usr/bin/env python3
import cv2
import rospy
import numpy as np
import time
import math
from MAV_ardupilot import MAV2

class fase1:
    def __init__(self, mav):
        self.mav = mav
        
        # velocidade de cruzeiro
        self.vel_cruzeiro = 0.5

        # altura do voo em relação ao tamanho incial da base costeira
        self.altitude = 0.5

    def run(self):
        self.mav.takeoff(self.altitude)
        rospy.sleep(7)

        # vai para o inicio do tubo
        self.mav.change_auto_speed(self.vel_cruzeiro)
        self.mav.go_to_local(-3, 3, 3.5, yaw=math.pi/2, sleep_time=10)
        rospy.loginfo("Iniciando scaneamento do tubo")

        


if __name__ == "__main__":
    rospy.init_node('fase1_tubo')
    mav = MAV2()
    missao = fase1(mav)
    missao.run()
    
