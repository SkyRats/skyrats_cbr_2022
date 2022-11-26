#!/usr/bin/env python3
from jinja2 import pass_eval_context
from baseDetector import CrossDetection
import cv2
import rospy
import numpy as np
import time
import math
from MAV_ardupilot import MAV2



TOL = 0.5 #tolerancia de erro das bases 
CAM_FOV = 1.64061 #94 graus
INIT_HEIGHT = 0.5


class fase1:
    def __init__(self, mav):
        self.mav = mav
        self.detection = detection

        # self.base_shortly = 0
        self.ja_mapeada = False  
        self.bases_visited = 0
        self.bases_stored = []   

        self.mav = mav
        
        # velocidade de cruzeiro
        self.vel_cruzeiro = 0.2

        # velocidade na qual o drone percorrerá o tubo
        self.vel_tubo = 0.1

        # altura do voo em relação ao tamanho incial da base costeira
        self.altitude = 1.5 - INIT_HEIGHT

    def centralize_on_cross(self, drone):
        TARGET = (int(drone.cam.shape[1]/2), int(drone.cam.shape[0]/2))

        is_centralized = False
        while not is_centralized:
            
            # Loop over frames to search for markers
            # If no markers were found, tries to shake the drone
            cross_detected = False
            timer = 0
            no_detection = 0
            
            while not cross_detected:
                              
                parameters = [[0, 0, 0], [255, 255, 255], [0, 0, 0]]

                frame = drone.cam
                list_of_bases = detection.base_detection(frame, parameters)

                if len(list_of_bases) > 0:
                    cross_detected = True
                    timer = 0

                if timer > 1000:
                    print("No visible bases, shaking drone...")
                    #drone.shake()
                    #self.mav.go_to_local(self.mav.drone_pose.pose.position.x, self.mav.drone_pose.pose.position.y, self.mav.drone_pose.pose.position.z + 0.5)
                    timer = 0
                    no_detection += 1

                if no_detection > 10:
                    print("Aruco not found...")
                    return

                timer += 1

            base = list_of_bases[0]

            # Calculate the PID errors
            delta_y =(TARGET[0] - base[0])*(1)
            delta_x = (TARGET[1] - base[1])*(1)

            # Adjust velocity
            drone.camera_pid(delta_x, delta_y, 0)

            # End centralization if the marker is close enough to the camera center
            if ((delta_x)**2 + (delta_y)**2)**0.5 < 40:
                drone.set_vel(0, 0, 0)
                is_centralized = True
                print(f"Centralized! x: {delta_x} y: {delta_y}")
                    
                return
    
    def find_potentials(image):
        contours, ret = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        shapes = []
        for contour in contours:
            approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)
            
            if len(approx) == 4 and cv2.arcLength(contour,True) > 200:
                shapes.append(approx)
        
        return shapes
    
    def base_detection(self, img, parameters):

        img_filter = detection.aply_filters(img, parameters)

        list_of_potentials = detection.find_potentials(img_filter)
        result = []
        for potential in list_of_potentials:

            if detection.verify(potential, img_filter):
                M = cv2.moments(potential)
                if M['m00'] != 0.0:
                    x = int(M['m10']/M['m00'])
                    y = int(M['m01']/M['m00'])
                    new_cross = True
                    tol = 3
                    for point in result:
                        if (point[0]-tol) <= x <= (point[0]+tol) and (point[1]-tol) <= y <= (point[1]+tol):
                            new_cross = False
                    if new_cross:
                        result.append((x,y))

        return result       

    def tube(self, mask_tube):
        pass
    
    
    # def mapping(self, x, y):
    #     j = 0  
    #     for j in range(self.bases_visited):
    #         print(self.bases_stored[j][1])
    #         print(self.bases_stored[j][2])
    #         if (abs(self.bases_stored[j][1] -x) - 0.2 <= TOL) and (abs(self.bases_stored[j][2] - y) - 0.2 <= TOL):
    #                 self.ja_mapeada = True
    #     if self.ja_mapeada == False:
    #         print("Base localizada: " + str(x) + " , " + str(y))
    #         #   self.mav.go_to_local(x, y, 3)
    #         self.mav.land()
    #         time.sleep(10)
    #         if self.bases_visited == 1:
    #             self.bases_stored.append(["A", -4.05, -0.55, 1.0])
    #             print("Base A visitada")
    #         elif self.bases_visited == 2:
    #             self.bases_stored.append(["B", -3.58, 7.02, 1.0])
    #             print("Base B visitada")
    #         elif self.bases_visited == 3:
    #             self.bases_stored.append(["C", self.mav.drone_pose.pose.position.x, self.mav.drone_pose.pose.position.y, self.mav.drone_pose.pose.position.z])
    #             print("Base C visitada")
    #         elif self.bases_visited == 4:
    #             self.bases_stored.append(["D", self.mav.drone_pose.pose.position.x, self.mav.drone_pose.pose.position.y, self.mav.drone_pose.pose.position.z])
    #             print("Base D visitada")
    #         elif self.bases_visited == 5:
    #             self.bases_stored.append(["E", self.mav.drone_pose.pose.position.x, self.mav.drone_pose.pose.position.y, self.mav.drone_pose.pose.position.z])
    #             print("Base E visitada")
    #         self.bases_visited += 1
    #         self.mav.takeoff(self.altitude) 
    #         rospy.sleep(7)       
    #         self.mav.go_to_local(self.mav.drone_pose.pose.position.x, self.mav.drone_pose.pose.position.y, self.altitude)
    #         self.mav.hold(2) 
    #     self.base_shortly = 0 
    #     self.ja_mapeada = False 

    def mapping(self):
        j = 0  
        #   self.mav.go_to_local(x, y, 3)
        self.mav.land()
        time.sleep(10)
        if self.bases_visited == 1:
            self.bases_stored.append(["A", -4.05, -0.55, 1.0])
            print("Base A visitada")
        elif self.bases_visited == 2:
            self.bases_stored.append(["B", -3.58, 7.02, 1.0])
            print("Base B visitada")
        elif self.bases_visited == 3:
            print("Base C visitada")
        elif self.bases_visited == 4:
            print("Base D visitada")
        elif self.bases_visited == 5:
            print("Base E visitada")
        self.bases_visited += 1
        self.mav.takeoff(1.5)
        rospy.sleep(7)       
        self.base_shortly = 0 

    def in_direction_bases(self):
        
        # current_x = self.mav.drone_pose.pose.position.x
        # current_y = self.mav.drone_pose.pose.position.y
        # while(np.sqrt((x - current_x  )**2 + (y - current_y)**2)) > TOL:
        #     if(self.base_shortly >= 500):
                
        #         current_x = self.mav.drone_pose.pose.position.x
        #         current_y = self.mav.drone_pose.pose.position.y
        #         bases_detected = self.base_detection(self.mav.cam, [[0, 0, 0], [255, 255, 255], [0, 0, 0]])
        #         print(bases_detected)
        #         for new_base in bases_detected:  
        #             self.centralize_on_cross(self.mav)
                    
        #             current_x = self.mav.drone_pose.pose.position.x
        #             current_y = self.mav.drone_pose.pose.position.y
        #             self.mapping(current_x, current_y)
        #     self.base_shortly += 1
        #     if (self.base_shortly == 10000):
        #         self.base_shortly = 500
        #     self.mav.set_position(x, y, z) 
 
        bases_detected = self.base_detection(self.mav.cam, [[0, 0, 0], [255, 255, 255], [0, 0, 0]])
        print(bases_detected)
        for new_base in bases_detected:  
            self.centralize_on_cross(self.mav)
            self.mapping()



    def land_known_base(self, x, y):
        self.mav.land()
        time.sleep(10)
        if x == self.bases_stored[1][1] and y == self.bases_stored[1][2]:
            self.bases_stored.append(["A", -4.05, -0.55, 1.0])
            print("Base A visitada")
        elif x == self.bases_stored[2][1] and y == self.bases_stored[2][2]:
            self.bases_stored.append(["B", -3.58, 7.02, 1.0])
            print("Base B visitada")
        self.bases_visited += 1 
        self.mav.takeoff(self.altitude)  
        rospy.sleep(7)       

    def in_direction_tube(self, x, y, z):
        pass
    
    def trajectory(self):
        self.mav.takeoff(self.altitude)
        rospy.sleep(7)
        self.mav.change_auto_speed(0.5)
        self.bases_stored.append(["0", 0, 0, INIT_HEIGHT])
        self.bases_visited += 1 
        self.bases_stored.append(["A", -4.05, -0.55, 1.0])
        self.bases_visited += 1 
        self.bases_stored.append(["B", -3.58, 7.02, 1.0])
        self.bases_visited += 1 


        ############ andar 2.30 ###########
        ######## largura 627 ####### 3 partes de 209
        ######## 0.045 ####
        ######## 2.135 
        ######## 4.225

        ######## comprimento 705.5 ###### 3 partes 235.166667
        ##### -0.675
        ##### -3.025
        ##### -5.375

        
        self.go_to_local(0, 0.045, self.altitude, yaw=math.pi/2, sleep_time=2)
        ############ quadrante 1 #############
        self.go_to_local(-3.025, 0.045, self.altitude, yaw=math.pi/2, sleep_time=7)
        self.in_direction_bases()
        ############ fim quadrante 1 #############
        ############ quadrante 2 #############
        self.go_to_local(-5.375, 2.135, self.altitude, yaw=math.pi/2, sleep_time=10)
        self.in_direction_bases()
        ############ fim quadrante 2 #############
        ############ quadrante 3 #############
        self.go_to_local(-3.025, 2.135, self.altitude, yaw=math.pi/2, sleep_time=7)
        self.in_direction_bases()
        ############ fim quadrante 3 #############
        ############ quadrante 4 #############
        self.go_to_local(-3.025, 4.225, self.altitude, yaw=math.pi/2, sleep_time=7)
        self.in_direction_bases()
        ############ fim quadrante 4 #############
        ############ quadrante 5 #############
        self.go_to_local(-0.675, 4.225, self.altitude, yaw=math.pi/2, sleep_time=7)
        self.in_direction_bases()
        ############ fim quadrante 5 #############
        ############ quadrante 6 #############
        self.go_to_local(-0.675, 2.135, self.altitude, yaw=math.pi/2, sleep_time=7)
        self.in_direction_bases()
         ############ fim quadrante 6 #############

        self.go_to_local(0, 0, self.altitude, yaw=math.pi/2, sleep_time=7)
        self.mav.land()

        # ######### Base aerea 1 ################
        # self.mav.go_to_local(1.5, 0, self.altitude + 1.5)
        # self.mav.go_to_local(2.75, 0.25, self.altitude + 1.5)
        # self.land_known_base()
        # self.mav.go_to_local(4.5, 0, self.altitude + 1.5)
        # self.mav.go_to_local(4.5, 0, self.altitude)
        # ######### fim base aerea 1 ################
        # self.in_direction_bases(6.5, 0, self.altitude)
        # self.in_direction_bases(6.5, -2, self.altitude)
        # self.in_direction_bases(0, -2, self.altitude)
        # self.in_direction_bases(0, -4, self.altitude)
        # self.in_direction_bases(6.5, -4, self.altitude)
        # self.in_direction_bases(6.5, -6, self.altitude)
        # self.in_direction_bases(1.5, -6, self.altitude)
        # ######### Base area 2 ################
        # self.mav.go_to_local(1.5, -6, self.altitude + 1.5)
        # self.mav.go_to_local(0.25,-6.25, self.altitude + 1.5)
        # self.land_known_base()
        # self.mav.go_to_local(0, -6, self.altitude + 1.5)
        # ######### fim base aerea 1 ################ 
        # self.mav.go_to_local(0, 0, self.altitude + 1.5)
        # self.mav.land()

if __name__ == "__main__":
    rospy.init_node('fase1')
    detection = CrossDetection()
    mav = MAV2()
    missao = fase1(mav, detection)
    missao.trajectory()
    
