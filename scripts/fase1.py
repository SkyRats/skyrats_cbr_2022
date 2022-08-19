#!/usr/bin/env python3
import cv2
import rclpy
import numpy as np
from pickle import FALSE
import time

import sys
import os
sys.path.insert(0,'/home/' + os.environ["USER"] + '/skyrats_ws2/src/mavbase2')
from MAV2 import MAV2
sys.path.insert(0,'/home/' + os.environ["USER"] + '/skyrats_ws2/src/marker_detection/scripts/CBRbase')
from crossdetection import find_potentials, aply_filters, verify

from tf_transformations import quaternion_from_euler, euler_from_quaternion


TOL = 0.5 #tolerancia de erro das bases 
CAM_FOV = 1.64061 #94 graus

class Base:
    def __init__(self, name, mav):
        self.mav = mav
        self._name = name
        self._cordenates = [self.mav.drone_pose.pose.position.x, self.mav.drone_pose.pose.position.y, self.mav.drone_pose.pose.position.z]

class fase1:
    def __init__(self, mav2):
        self.mav2 = mav2

        self.bases_visited = 0
        self.bases_stored = []   
        self.ja_mapeada = False  
        self.vel = 0.5

        self.tube_found = False
        self.fim_encontrado = False
        self.cx_tube = self.cy_tube = self.tw = self.th = 0.0
        self.rows = self.cols = 0

    def land_on_cross(self):
        target = (int(self.mav2.cam.shape[0]/2), int(self.mav2.cam.shape[1]/2))
        is_centralize = False
        while not is_centralize:
            # Loop over frames to search for markers
            # If no markers were found, return
            timer = 0
            cross_detected = False
            while not cross_detected:
                frame = self.mav2.cam
                list_of_bases = self.base_detection(self.mav2.cam, [[0, 0, 0], [255, 255, 255], [0, 0, 0]])

                if len(list_of_bases) > 0:
                    cross_detected = True
                    timer = 0

                if timer > 10000:
                    print("Cross not found...")
                    return

                timer += 1
                rclpy.spin_once(self.mav2)

            pixel = self.base_detection(self.mav2.cam, [[0, 0, 0], [255, 255, 255], [0, 0, 0]])

            dists = []
            for i in pixel:
                w, h, z = self.mav2.cam.shape
                image_center_x = w/2 
                image_center_y = h/2

                dif_x = i[0] - image_center_x # dif_x = diferenca entre o centro da imagem e o centro do barco
                dif_y = i[1] - image_center_y
                dist = (dif_x**2 + dif_y**2)**(1/2)
                dists.append(dist)

            min_dist = min(dists)
            
            for i in pixel:
                dif_x = i[0] - image_center_x # dif_x = diferenca entre o centro da imagem e o centro do barco
                dif_y = i[1] - image_center_y
                dist = (dif_x**2 + dif_y**2)**(1/2)
                if min_dist == dist:

                    # Calculate the PID errors
                    delta_x = target[0] - i[0]
                    delta_y = target[1] - i[1]
                    delta_area = 0

                    # Adjust velocity
                    self.mav2.camera_pid(delta_x, delta_y, delta_area)

                    # End centralization if the marker is close enough to the camera center
                    if ((delta_x)*2 + (delta_y)*2)*0.5 < 30:
                        self.mav2.set_vel(0, 0, 0)
                        is_centralize = True
                        print(f"Centralized! x: {delta_x} y: {delta_y}")
                
            rclpy.spin_once(self.mav2)

        self.mav2.land()
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

        img_filter = aply_filters(img, parameters)

        list_of_potentials = find_potentials(img_filter)

        result = []
        for potential in list_of_potentials:

            if verify(potential, img_filter):
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
        current_x = self.mav2.drone_pose.pose.position.x
        current_y = self.mav2.drone_pose.pose.position.y
        
        dist_real = np.tan(CAM_FOV/2) * self.mav2.drone_pose.pose.position.z
        w, h, z = mask_tube.shape
        one_pixel_in_meters = dist_real/(w/2)
        image_center_x = w/2 
        image_center_y = h/2

        dif_x = self.cx_tube - image_center_x # dif_x = diferenca entre o centro da imagem e o centro do barco
        dif_y = self.cy_tube  - image_center_y

        meters_y = dif_x * one_pixel_in_meters  # Conversao da diferenca em pixels para metros
        meters_x = dif_y * one_pixel_in_meters

        cord_base_x = current_x - meters_x #Coordenada da base movel
        cord_base_y = current_y - meters_y
        self.mav2.go_to_local(cord_base_x, cord_base_y , 3)
        time.sleep(2)
        self.mav2.go_to_local(cord_base_x, cord_base_y , 1)
        hight_saved_x = self.mav2.drone_pose.pose.position.x
        hight_saved_y = self.mav2.drone_pose.pose.position.y

        _, threshold = cv2.threshold(mask_tube, 0, 10, cv2.THRESH_BINARY)
        
        contours, _ = cv2.findContours(
            threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            if i == 0:
                i = 1
                continue
            # cv2.approxPloyDP() function to approximate the shape
            approx = cv2.approxPolyDP(
                contour, 0.01 * cv2.arcLength(contour, True), True)
            
            if len(approx) == 4:
                M = cv2.moments(contour)
                try: 
                    self.cx_tube = int(M['m10']/M['m00']) #Passa o ponto do frame que foram encontradas os barcos
                    self.cy_tube = int(M['m01']/M['m00'])
                except ZeroDivisionError:
                    continue
                (cx, cy, self.tw, self.th) = cv2.boundingRect(approx)   

        atan_vel = self.tw/self.th

        hight_i = (self.th**2 +self.tw**2)/2

        self.mav2.go_to_local(cord_base_x, cord_base_y , 1)
        self.mav2.rotate(atan_vel)

        while self.fim_encontrado == False:
            self.mav2.set_vel(self.vel, 0,0)
            self.rows, self.cols, b = mask_tube.shape
            for i in range(self.cols):
                if mask_tube[self.rows - 200, i] == [255, 255, 255]:
                    self.fim_encontrado = False
                    i = self.cols + 1
                else: 
                    self.fim_encontrado = True
            cv2.imshow("output", np.hstack([self.mav2.cam, mask_tube])) #teste
            cv2.waitKey(0)
            
        
        contours, _ = cv2.findContours(
            threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            if i == 0:
                i = 1
                continue
            # cv2.approxPloyDP() function to approximate the shape
            approx = cv2.approxPolyDP(
                contour, 0.01 * cv2.arcLength(contour, True), True)
            
            if len(approx) == 4:
                M = cv2.moments(contour)
                try: 
                    self.cx_tube = int(M['m10']/M['m00']) #Passa o ponto do frame que foram encontradas os barcos
                    self.cy_tube = int(M['m01']/M['m00'])
                except ZeroDivisionError:
                    continue
                (cx, cy, self.tw, self.th) = cv2.boundingRect(approx)  

        hight_f = (self.th**2 +self.tw**2)/2
        hight_final_x = self.mav2.drone_pose.pose.position.x
        hight_final_y = self.mav2.drone_pose.pose.position.y
        hight_total = hight_i + hight_f + np.sqrt((hight_final_x-hight_saved_x)**2 + (hight_final_y - hight_saved_y)**2 )
        self.tube_found = True
        print("tube hight: " + hight_total + "m")
        print("tube width: " + self.tw + "m")

        self.mav2.go_to_local(current_x, current_y , 3)
        self.tube_found = True    
 
    def mapping(self, x, y):
        rclpy.spin_once(self.mav2)
        current_x = self.mav2.drone_pose.pose.position.x
        current_y = self.mav2.drone_pose.pose.position.y
        for j in range(self.bases_visited  - 1):
            if abs(self.bases_stored[j].cordenates[0] - x) <= TOL:
                if abs(self.bases_stored[j].cordenates[1] - y) <= TOL:
                    self.ja_mapeada = True
        if self.ja_mapeada == False:
            print("Base localizada: " + str(x) + " , " + str(y))
            self.mav2.go_to_local(x, y, 3)
            self.land_on_cross()
            if self.bases_visited == 1:
                self.bases_stored.append(Base("A", self.mav2))
            elif self.bases_visited == 2:
                self.bases_stored.append(Base("B", self.mav2))
            elif self.bases_visited == 3:
                self.bases_stored.append(Base("C", self.mav2))
            elif self.bases_visited == 4:
                self.bases_stored.append(Base("D", self.mav2))
            elif self.bases_visited == 5:
                self.bases_stored.append(Base("E", self.mav2))
            self.bases_visited += 1
            time.sleep(10)
            self.mav2.takeoff(3 - self.mav2.drone_pose.pose.position.z) #verificar se eh o quanto sobe ou em relacao ao inicio
            time.sleep(4)
            self.mav2.go_to_local(current_x, current_y, 3) 
        self.ja_mapeada = False

    def in_direction_bases(self, x, y, z):
        rclpy.spin_once(self.mav2)
        current_x = self.mav2.drone_pose.pose.position.x
        current_y = self.mav2.drone_pose.pose.position.y
        while(np.sqrt((x - current_x  )**2 + (y - current_y)**2)) > TOL:
            rclpy.spin_once(self.mav2)
            current_x = self.mav2.drone_pose.pose.position.x
            current_y = self.mav2.drone_pose.pose.position.y
            #print(self.mav2.drone_pose.pose.position.x)

            bases_detected = self.base_detection(self.mav2.cam, [[0, 0, 0], [255, 255, 255], [0, 0, 0]])

            for new_base in bases_detected:                     
                dist_real = np.tan(CAM_FOV/2) * self.mav2.drone_pose.pose.position.z
                w, h, z = self.mav2.cam.shape
                one_pixel_in_meters = dist_real/(w/2)
                image_center_x = w/2 
                image_center_y = h/2

                dif_x = new_base[0] - image_center_x # dif_x = diferenca entre o centro da imagem e o centro do barco
                dif_y = new_base[1] - image_center_y

                meters_y = dif_x * one_pixel_in_meters  # Conversao da diferenca em pixels para metros
                meters_x = dif_y * one_pixel_in_meters

                cord_base_x = current_x + meters_x #Coordenada da base movel
                cord_base_y = current_y - meters_y
                self.mapping(cord_base_x, cord_base_y)
            self.mav2.set_position(x, y, z, vel_xy = 1.0)
            time.sleep(2)


    def in_direction_tube(self, x, y, z):
        rclpy.spin_once(self.mav2)
        current_x = self.mav2.drone_pose.pose.position.x
        current_y = self.mav2.drone_pose.pose.position.y
        while(np.sqrt((x - current_x  )**2 + (y - current_y)**2)) > TOL:
            rclpy.spin_once(self.mav2)
            current_x = self.mav2.drone_pose.pose.position.x
            current_y = self.mav2.drone_pose.pose.position.y
            #print(self.mav2.drone_pose.pose.position.y)
    
            img = np.asarray(self.mav2.cam)
            hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

            lower_orange = np.array([5,50,50])
            upper_orange = np.array([15,255,255])
            mask_tube= cv2.inRange(hsv, lower_orange, upper_orange)

            # _, threshold = cv2.threshold(mask_tube, 0, 10, cv2.THRESH_BINARY)
            
            # contours, _ = cv2.findContours(
            #     threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            # for contour in contours:
            #     if i.any() == 0:
            #         i = 1
            #         continue
            
            #     # cv2.approxPloyDP() function to approximate the shape
            #     approx = cv2.approxPolyDP(
            #         contour, 0.01 * cv2.arcLength(contour, True), True)
                
            #     if len(approx) == 4:
            #         M = cv2.moments(contour)
            #         try: 
            #             self.cx_tube = int(M['m10']/M['m00']) #Passa o ponto do frame que foram encontradas os barcos
            #             self.cy_tube = int(M['m01']/M['m00'])
            #         except ZeroDivisionError:
            #             continue
            #         (cx, cy, self.tw, self.th) = cv2.boundingRect(approx)  
            #         tube_center = (self.cx_tube, self.cy_tube)
            
            # cv2.imshow("output", np.hstack([self.mav2.cam, mask_tube])) #teste
            # cv2.waitKey(0)

            # if tube_center != (0, 0):
            #     #self.tube(mask_tube)
            #     print("tubo encontrado")
            self.mav2.set_position(x, y, z, vel_xy = 1.0)
    
    def trajectory(self):
        self.mav2.takeoff(3 - self.mav2.drone_pose.pose.position.z)
        self.bases_stored.append(Base("0", self.mav2))
        self.bases_visited += 1;
        self.mav2.go_to_local(0, 1, 3)
        print("cheguei no 0,1,3")
        print(self.mav2.drone_pose.pose.position.x)

        self.in_direction_bases(-6, 1, 3)
        self.in_direction_bases(-6, 5, 3)
        self.in_direction_bases(0, 5, 3)
        self.in_direction_tube(0, 1, 3)
        
        self.mav2.go_to_local(0, 0, 3)
        self.precision_land()
        


if __name__ == "__main__":
    rclpy.init()
    mav = MAV2()
    missao = fase1(mav)
    missao.trajectory()
    
