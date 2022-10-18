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

    def trajectory(self):
        self.mav2.change_auto_speed(0.8)
        self.detection.qr_debug = False
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
                qr_result = self.detection.qrdetection(self.mav2.cam)
                rospy.logwarn("QR data: " + str(qr_result))
                if not self.detection.detected:
                    rospy.logwarn("Trying to detect again...")
                    self.mav2.go_to_local(self.bases_not_visited[i][0],self.bases_not_visited[i][1], self.bases_not_visited[i][2] + 1)
                    qr_result = self.detection.qrdetection(self.mav2.cam)
                    rospy.logwarn("QR data: " + str(qr_result))
                if not self.detection.detected:
                    rospy.logwarn("Trying to detect again...")
                    self.mav2.go_to_local(self.bases_not_visited[i][0],self.bases_not_visited[i][1], self.bases_not_visited[i][2] + 0.3)
                    qr_result = self.detection.qrdetection(self.mav2.cam)
                    rospy.logwarn("QR data: " + str(qr_result))
                self.detection.detected = False
                self.bases_visited.append(self.bases_not_visited[i]) 
                qtdade_bases_visited += 1
        rclpy.spin_once(self.mav2) 
        self.mav2.go_to_local(0, 0, 2)
        self.mav2.land()
        rospy.logwarn("QRs detected: " + str(self.detection.qrs))

    def trajectory_test(self):
        self.mav2.change_auto_speed(0.5)
        self.detection.qr_debug = False
        self.mav2.takeoff(1)

        self.mav2.go_to_local(1, 1, 1)
        self.mav2.go_to_local(1, 1, 0.5)
        cam = cv2.VideoCapture(0)
        qr_result = self.detection.qrdetection(cam)
        rospy.logwarn("QR data: " + str(qr_result))
        if not self.detection.detected:
            rospy.logwarn("Trying to detect again...")
            self.mav2.go_to_local(1, 1, 0.7)
            qr_result = self.detection.qrdetection(cam)
            rospy.logwarn("QR data: " + str(qr_result))
        if not self.detection.detected:
            rospy.logwarn("Trying to detect again...")
            self.mav2.go_to_local(1, 1, 0.3)
            qr_result = self.detection.qrdetection(cam)
            rospy.logwarn("QR data: " + str(qr_result))
        self.mav2.go_to_local(1, 1, 1)
        self.mav2.go_to_local(0, 0, 1)
        self.mav2.land()
        rospy.logwarn("QRs detected: " + str(self.detection.qrs))


if __name__ == "__main__":
    rclpy.init()
    mav = MAV2()
    missao = fase4(mav)
    missao.trajectory_test()
    #missao.trajectory()
    
