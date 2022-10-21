#!/usr/bin/env python3
import cv2
import numpy as np
import sys
import os
#sys.path.insert(0,'/home/' + os.environ["USER"] + '/skyrats_ws2/src/mavbase2')
#from MAV2 import MAV2
from MAV_ardupilot import MAV2
import time
#import rclpy
import rospy
#from rclpy.clock import Clock
from qrDetection import QRDetection

CAM_FOV = 1.64061 #94 graus

class Base:
    def __init__(self, name, mav, x, y, z):
        self.mav = mav
        self._name = name
        self._coordinates = [x, y, z]
        self._package = None

class fase4:
    def __init__(self, mav2):
        super().__init__()
        self.mav2 = mav2
        self.coastbase_coords = []
        self.bases_not_visited = []
        self.bases_visited = []
        self.destination = None
        self.detection = QRDetection()
        self.detection.cam = mav2.cam
        self.cam_id = 0
    """
    def trajectory(self):
        self.detection.qr_debug = False
        self.mav2.change_auto_speed(0.8)
        self.mav2.takeoff(2)
        qtdade_bases_visited = 0

        self.bases_not_visited.append([-2.5, 0, 1])
        self.bases_not_visited.append([-4, 2, 0])
        self.bases_not_visited.append([-6, 0, 0])
        self.bases_not_visited.append([-2, 5, 0])
        self.bases_not_visited.append([0.31, 6.1, 1.55])

        for i in range(len(self.bases_not_visited)):
            
            if self.bases_not_visited[i] not in self.bases_visited:

                self.mav2.go_to_local(self.bases_not_visited[i][0],self.bases_not_visited[i][1], 2)
                self.mav2.go_to_local(self.bases_not_visited[i][0],self.bases_not_visited[i][1], self.bases_not_visited[i][2] + 0.5)
                capture = cv2.VideoCapture(0)
                qr_result = self.detection.qrdetection(capture)
                rospy.logwarn("QR data: " + str(qr_result))
                if not self.detection.detected:
                    rospy.logwarn("Trying to detect again...")
                    # Tries to detect from lower altitude
                    self.mav2.go_to_local(self.bases_not_visited[i][0],self.bases_not_visited[i][1], self.bases_not_visited[i][2] + 0.4)
                    qr_result = self.detection.qrdetection(self.mav2.cam)
                    rospy.logwarn("QR data: " + str(qr_result))
                if not self.detection.detected:
                    rospy.logwarn("Trying to detect again...")
                    # Tries to detect from higher altitude
                    self.mav2.go_to_local(self.bases_not_visited[i][0],self.bases_not_visited[i][1], self.bases_not_visited[i][2] + 1)
                    qr_result = self.detection.qrdetection(self.mav2.cam)
                    rospy.logwarn("QR data: " + str(qr_result))
                self.detection.detected = False
                self.bases_visited.append(self.bases_not_visited[i]) 
                qtdade_bases_visited += 1
        rclpy.spin_once(self.mav2) 
        self.mav2.go_to_local(0, 0, 2)
        self.mav2.land()
        rospy.logwarn("QRs detected: " + str(self.detection.qrs))
    """
    def trajectory(self):

        base1=(-4.3, -0.6, 0.5)
        base2=(-5.875, -0.05, -0.5)
        base3=(-2.15, 1.81, -0.5)
        base4=(-5.97, 4.87, 0.5)
        base5=(-3.22, 4.97, -0.5)

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
            self.mav.go_to_local(self.bases_not_visited[qtdade_bases_visited][0],self.bases_not_visited[qtdade_bases_visited][1], self.bases_not_visited[qtdade_bases_visited][2] + 0.5,  yaw=math.pi/2, sleep_time=3)
            
            # QR DETECTION
            capture = cv2.VideoCapture(0)
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            rospy.loginfo("Started QR detection")
            qr_result = self.detection.qrdetection(capture)
            rospy.logwarn("QR data: " + str(qr_result))
            if not self.detection.detected:
                rospy.logwarn("Trying to detect again...")
                # Tries to detect from higher altitude
                self.mav2.go_to_local(self.bases_not_visited[i][0],self.bases_not_visited[i][1], self.bases_not_visited[i][2] + 0.7)
                qr_result = self.detection.qrdetection(self.mav2.cam)
                rospy.logwarn("QR data: " + str(qr_result))
            self.detection.detected = False

            self.mav.go_to_local(self.bases_not_visited[qtdade_bases_visited][0],self.bases_not_visited[qtdade_bases_visited][1], self.altitude,  yaw=math.pi/2, sleep_time=4)
            qtdade_bases_visited += 1
        rospy.logwarn("QRs detected: " + str(self.detection.qrs))
        rospy.logwarn("FINALIZADO")
        self.mav.go_to_local(0, 0, self.altitude, yaw=math.pi/2, sleep_time=15)
        self.mav.land()

    def trajectory_test(self):
        #self.mav2.change_auto_speed(0.5)
        self.detection.qr_debug = False
        self.mav2.takeoff(1)

        #self.mav2.go_to_local(-1, 1, 1)
        #self.mav2.go_to_local(-1, 1, 0.5)
        self.mav2.go_to_local(-2.15, 1.81, 0)  # base 3
        cam = cv2.VideoCapture(0)
        qr_result = self.detection.qrdetection(cam)
        rospy.logwarn("QR data: " + str(qr_result))

        if not self.detection.detected:
            rospy.logwarn("Trying to detect again...")
            self.mav2.go_to_local(-2.15, 1.81, 0.2)
            qr_result = self.detection.qrdetection(cam)
            rospy.logwarn("QR data: " + str(qr_result))
            qr_result = self.detection.qrdetection(cam)
            rospy.logwarn("QR data: " + str(qr_result))
        self.mav2.go_to_local(-2.15, 1.81, 1)
        self.mav2.go_to_local(0, 0, 1)
        self.mav2.land()
        rospy.logwarn("QRs detected: " + str(self.detection.qrs))


if __name__ == "__main__":
    rclpy.init()
    mav = MAV2()
    missao = fase4(mav)
    missao.trajectory_test()
    #missao.trajectory()
    
