#!/usr/bin/env python3
import rospy
import numpy as np
import time
import math
from MAV_ardupilot import MAV2

class return_to_home:
    def __init__(self, mav): 
        self.mav = mav
        
        # velocidade de cruzeiro
        self.vel_cruzeiro = 0.5

    def come_back(self):
        self.mav.takeoff(1.0)
        time.sleep(7)
        self.mav.change_auto_speed(self.vel_cruzeiro)
        self.mav.go_to_local(0, 0, 1.0, yaw=math.pi/2, sleep_time=15)
        self.mav.land()
        time.sleep(10)

if __name__ == "__main__":
    rospy.init_node('go back')
    mav = MAV2()
    missao = return_to_home(mav)
    missao.come_back()
