#!/usr/bin/env python3
import cv2
import numpy as np
import sys
import os
sys.path.insert(0,'/home/' + os.environ["USER"] + '/skyrats_ws2/src/mavbase2')
from MAV2 import MAV2
#from MAV_ardupilot import MAV2
import time
import rclpy
from rclpy.clock import Clock
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
        self.detection.frame = mav2.cam
        self.cam_id = 0

    def trajectory(self):
        self.mav2.change_auto_speed(0.8)
        self.detection.qr_debug = False
        rclpy.spin_once(self.mav2) 
        self.mav2.takeoff(2)
        current_x = self.mav2.drone_pose.pose.position.x
        current_y = self.mav2.drone_pose.pose.position.y
        dist_menor=8*(2**(1/2))
        qtdade_bases_visited = 0

        self.bases_not_visited.append([-2.5, 0, 1])
        self.bases_not_visited.append([-4, 2, 0])
        self.bases_not_visited.append([-6, 0, 0])
        self.bases_not_visited.append([-2, 5, 0])
        self.bases_not_visited.append([0.31, 6.1, 1.55])

        while(qtdade_bases_visited!=5):
            rclpy.spin_once(self.mav2) 
            current_x = self.mav2.drone_pose.pose.position.x
            current_y = self.mav2.drone_pose.pose.position.y
            for i in range(len(self.bases_not_visited)):
                
                if self.bases_not_visited[i] not in self.bases_visited:
                  
                    X=self.bases_not_visited[i][0]
                    Y=self.bases_not_visited[i][1]

                    dist=np.sqrt((X- current_x)**2 + (Y - current_y)**2)

                    if dist<dist_menor:
                        dist_menor=dist
                        i_oficial=i
            dist_menor = 8*(2**(1/2))
            self.mav2.go_to_local(self.mav2.drone_pose.pose.position.x, self.mav2.drone_pose.pose.position.y, 2)
            self.mav2.go_to_local(self.bases_not_visited[i_oficial][0],self.bases_not_visited[i_oficial][1], 2)
            self.mav2.go_to_local(self.bases_not_visited[i_oficial][0],self.bases_not_visited[i_oficial][1], self.bases_not_visited[i_oficial][2] + 0.5)
            qr_result = self.detection.qrdetection(self.mav2.cam)
            self.mav2.get_logger().warn("QR data: " + str(qr_result))
            if not self.detection.detected:
                self.mav2.get_logger().warn("Trying to detect again...")
                self.mav2.go_to_local(self.mav2.drone_pose.pose.position.x, self.mav2.drone_pose.pose.position.y, self.mav2.drone_pose.pose.position.z + 0.2, 0, 0.1)
                qr_result = self.detection.qrdetection(self.mav2.cam)
                self.mav2.get_logger().warn("QR data: " + str(qr_result))
            self.detection.detected = False
            self.bases_visited.append(self.bases_not_visited[i_oficial]) 
            qtdade_bases_visited += 1
        rclpy.spin_once(self.mav2) 
        current_x = self.mav2.drone_pose.pose.position.x
        current_y = self.mav2.drone_pose.pose.position.y    
        self.mav2.go_to_local(current_x, current_y, 2)
        self.mav2.go_to_local(0, 0, 2)
        self.mav2.land()
        self.mav2.get_logger().warn("QRs detected: " + str(self.detection.qrs))

if __name__ == "__main__":
    rclpy.init()
    mav = MAV2()
    missao = fase4(mav)
    missao.trajectory()
    
