    #!/usr/bin/env python3
import cv2
import rclpy
import numpy as np
from pickle import FALSE
import time

from baseDetector import CrossDetection
import sys
import os
sys.path.insert(0,'/home/' + os.environ["USER"] + '/skyrats_ws2/src/mavbase2')
from MAV2 import MAV2
sys.path.insert(0,'/home/' + os.environ["USER"] + '/skyrats_ws2/src/marker_detection/scripts/CBRbase')
from crossdetection import find_potentials, aply_filters, verify

from tf_transformations import quaternion_from_euler, euler_from_quaternion


TOL = 0.5 #tolerancia de erro das bases 
CAM_FOV = 1.64061 #94 graus
init_height = 0.5

# [self.mav.drone_pose.pose.position.x, self.mav.drone_pose.pose.position.y, self.mav.drone_pose.pose.position.z]

class fase1:
    def __init__(self, mav2):
        self.mav2 = mav2

        self.bases_visited = 0
        self.bases_stored = []   
        self.base_shortly = 0
        self.ja_mapeada = False  
        self.vel = 0.5


        self.tube_found = False
        self.fim_encontrado = False
        self.cx_tube = self.cy_tube = self.tw = self.th = 0.0
        self.rows = self.cols = 0

    def centralize_on_cross(self, drone):

        detection = CrossDetection()

        TARGET = (int(drone.cam.shape[1]/2), int(drone.cam.shape[0]/2))

        is_centralized = False
        while not is_centralized:
            
            # Loop over frames to search for markers
            # If no markers were found, tries to shake the drone
            cross_detected = False
            timer = 0
            no_detection = 0
            
            while not cross_detected:
                rclpy.spin_once(drone)
                
                parameters = [[0, 0, 0], [255, 255, 255], [0, 0, 0]]

                frame = drone.cam
                list_of_bases = detection.base_detection(frame, parameters)

                if len(list_of_bases) > 0:
                    cross_detected = True
                    timer = 0

                if timer > 1000:
                    print("No visible bases, shaking drone...")
                    drone.shake()
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
        one_pixel_in_meters = dist_real/(self.w/2)


        dif_x = self.cx_tube - self.image_center_x # dif_x = diferenca entre o centro da imagem e o centro do barco
        dif_y = self.cy_tube  - self.image_center_y

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
        current_z = self.mav2.drone_pose.pose.position.z
        j = 0  
        for j in range(self.bases_visited):
            print(self.bases_stored[j][1])
            print(self.bases_stored[j][2])
            if (abs(self.bases_stored[j][1] -x) - 0.2 <= TOL) and (abs(self.bases_stored[j][2] - y) - 0.2 <= TOL):
                    self.ja_mapeada = True
        if self.ja_mapeada == False:
            self.mav2.hold(2)   
            print("Base localizada: " + str(x) + " , " + str(y))
            #   self.mav2.go_to_local(x, y, 3)
            self.mav2.land()
            rclpy.spin_once(self.mav2)
            if self.bases_visited == 1:
                self.bases_stored.append(["A", self.mav2.drone_pose.pose.position.x, self.mav2.drone_pose.pose.position.y, self.mav2.drone_pose.pose.position.z])
                print("Base A visitada")
            elif self.bases_visited == 2:
                self.bases_stored.append(["B", self.mav2.drone_pose.pose.position.x, self.mav2.drone_pose.pose.position.y, self.mav2.drone_pose.pose.position.z])
                print("Base B visitada")
            elif self.bases_visited == 3:
                self.bases_stored.append(["C", self.mav2.drone_pose.pose.position.x, self.mav2.drone_pose.pose.position.y, self.mav2.drone_pose.pose.position.z])
                print("Base C visitada")
            elif self.bases_visited == 4:
                self.bases_stored.append(["D", self.mav2.drone_pose.pose.position.x, self.mav2.drone_pose.pose.position.y, self.mav2.drone_pose.pose.position.z])
                print("Base D visitada")
            elif self.bases_visited == 5:
                self.bases_stored.append(["E", self.mav2.drone_pose.pose.position.x, self.mav2.drone_pose.pose.position.y, self.mav2.drone_pose.pose.position.z])
                print("Base E visitada")
            self.bases_visited += 1
            time.sleep(2)
            rclpy.spin_once(self.mav2)
            self.mav2.takeoff(current_z - self.mav2.drone_pose.pose.position.z) #verificar se eh o quanto sobe ou em relacao ao inicio
            self.mav2.hold(2) 
        self.base_shortly = 0 
        self.ja_mapeada = False 

    def in_direction_bases(self, x, y, z):
        rclpy.spin_once(self.mav2)
        current_x = self.mav2.drone_pose.pose.position.x
        current_y = self.mav2.drone_pose.pose.position.y
        while(np.sqrt((x - current_x  )**2 + (y - current_y)**2)) > TOL:
            if(self.base_shortly >= 500):
                rclpy.spin_once(self.mav2)
                current_x = self.mav2.drone_pose.pose.position.x
                current_y = self.mav2.drone_pose.pose.position.y
                bases_detected = self.base_detection(self.mav2.cam, [[0, 0, 0], [255, 255, 255], [0, 0, 0]])
                print(bases_detected)
                for new_base in bases_detected:  
                    self.centralize_on_cross(self.mav2)
                    rclpy.spin_once(self.mav2)
                    current_x = self.mav2.drone_pose.pose.position.x
                    current_y = self.mav2.drone_pose.pose.position.y
                    self.mapping(current_x, current_y)
            self.base_shortly += 1
            if (self.base_shortly == 10000):
                self.base_shortly = 5000
            self.mav2.set_position(x, y, z) 

    def in_direction_tube(self, x, y, z):
        rclpy.spin_once(self.mav2)
        current_x = self.mav2.drone_pose.pose.position.x
        current_y = self.mav2.drone_pose.pose.position.y
        while(np.sqrt((x - current_x  )**2 + (y - current_y)**2)) > TOL:
            self.mav2.set_position(x, y, z)
    
    def trajectory(self):
        c_height = 2 - init_height
        self.mav2.takeoff(c_height)
        self.mav2.go_to_local(0, 0, c_height)
        self.mav2.change_auto_speed(1.0)
        self.bases_stored.append(["0", 0, 0, self.mav2.drone_pose.pose.position.z])
        self.bases_visited += 1 
        print(self.mav2.drone_pose.pose.position.x)
        
        self.in_direction_bases(1.5, 0, c_height)
        self.mav2.go_to_local(1.5, 0, c_height + 1.5)
        self.in_direction_bases(4.5, 0, c_height + 1.5)
        self.mav2.go_to_local(4.5, 0, c_height)
        self.in_direction_bases(6.5, 0, c_height)
        self.in_direction_bases(6.5, -2, c_height)
        self.in_direction_bases(0, -2, c_height)
        self.in_direction_bases(0, -4, c_height)
        self.in_direction_bases(6.5, -4, c_height)
        self.in_direction_bases(6.5, -6, c_height)
        self.in_direction_bases(1.5, -6, c_height)
        self.mav2.go_to_local(1.5, -6, c_height + 1.5)
        self.in_direction_bases(0, -6, c_height + 1.5)   
        self.mav2.go_to_local(0, 0, c_height + 1.5)
        self.mav2.land()

if __name__ == "__main__":
    rclpy.init()
    mav = MAV2()
    missao = fase1(mav)
    rclpy.spin_once(mav)
    current_x = mav.drone_pose.pose.position.x
    current_y = mav.drone_pose.pose.position.y
    if current_x >= TOL or current_y >= TOL:
        mav.go_to_local(0, 0, 3)
    else:
        # missao.teste()
        missao.trajectory()
    
