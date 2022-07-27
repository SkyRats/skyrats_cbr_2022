#!/usr/bin/env python3

import cv2
import rclpy
import numpy as np
from MAV2 import MAV2
from pickle import FALSE
import time

TOL = 0.5 #tolerancia de erro das bases 

class base:
    def __init__(self, name):
        self._name = name
        self._coordenate = [self.mav2.drone_pose.pose.position.x, self.mav2.drone_pose.pose.position.y, self.mav2.drone_pose.pose.position.z]

class fase1:
    def __init__(self,mav):
        self.mav2 = MAV2

        self.bases_visited = 0
        self.bases_stored = []   
        self.ja_mapeada = False  
        self.vel = 0.2

        self.tube_found = False
        self.fim_encontrado = False
        self.cx_tube, self.cy_tube, self.tw, self.th = 0.0
        self.rows, self.cols = 0

        


    #def precision_land(self):   #Pousa na cruz utilizando conrole proporcional
  
    def tube(self, mask_tube): 
        cord_x = self.mav2.drone_pose.pose.position.x
        cord_y = self.mav2.drone_pose.pose.position.y
        one_pixel_in_meters = 0.0168      #Foi visto empiricamente que a uma altura de 4 metros cada pixel da imagem equivale a 0,28 metros
        h, w, c = mask_tube.shape
        image_center_x = w/2 
        image_center_y = h/2

        dif_x = i[0] - image_center_x # dif_x = diferenca entre o centro da imagem e o centro do barco
        dif_y = i[1] - image_center_y

        meters_y = dif_x * one_pixel_in_meters  # Conversao da diferenca em pixels para metros
        meters_x = dif_y * one_pixel_in_meters

        cord_base_x = cord_x - meters_x #Coordenada da base movel
        cord_base_y = cord_y - meters_y
        self.mav2.go_to_local(cord_base_x, cord_base_y , 3)
        time.sleep(2)
        self.mav2.go_to_local(cord_base_x, cord_base_y , 1)
        
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
                # compute the bounding box of the contour and use the
                # bounding box to compute the aspect ratio
                (self.cx_tube, self.cy_tube, self.tw, self.th) = cv2.boundingRect(approx)   

        atan_vel = self.tw/self.th
        hight_i = self.th/2

        self.mav2.go_to_local(cord_base_x, cord_base_y , 1)
        
        while self.fim_encontrado == False:
            self.mav2.set_vel(self.vel * np.sin(atan_vel), self.vel * np.cos(atan_vel),0,0,0,0)
            self.rows, self.cols, b = self.mask_tube.shape
            for i in range(self.cols):
                if mask_tube[self.rows - 200, i] == [255, 255, 255]:
                    self.fim_encontrado = False
                    i = self.cols + 1
                else: 
                    self.fim_encontrado = True
            dist_p += 0.2
        
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
                # compute the bounding box of the contour and use the
                # bounding box to compute the aspect ratio
                (self.cx_tube, self.cy_tube, self.tw, self.th) = cv2.boundingRect(approx)   

        hight_f = self.th/2
        hight_total = hight_i + hight_f + dist_p # mostrar o width e o height
        self.tube_found = True

        self.mav2.go_to_local(cord_x, cord_y , 3)

        self.tube_found = True     
    
    def mapping(self):
        cord_x = self.mav2.drone_pose.pose.position.x
        cord_y = self.mav2.drone_pose.pose.position.y
        hsv = cv2.cvtColor(self.mav2.cam,cv2.COLOR_BGR2HSV)

        #alterar para encontrar plataformas
        lower_blue = np.array([110,50,50])
        upper_blue = np.array([130,255,255])
        mask_BasePossible = cv2.inRange(hsv, lower_blue, upper_blue)

        base_circles = cv2.HoughCircles(mask_BasePossible,cv2.HOUGH_GRADIENT,1,20,
                            param1=50,param2=30,minRadius=0,maxRadius=0)

        base_circles = np.uint16(np.around(base_circles))
        for i in base_circles[0,:]:
            # draw the outer circle
            cv2.circle(mask_BasePossible,(i[0],i[1]),i[2],(0,255,0),2)
            # draw the center of the circle
            cv2.circle(mask_BasePossible,(i[0],i[1]),2,(0,0,255),3)
            cv2.waitKey(15)

            one_pixel_in_meters = 0.0168      #Foi visto empiricamente que a uma altura de 4 metros cada pixel da imagem equivale a 0,28 metros
            h, w, c = mask_BasePossible.shape
            image_center_x = w/2 
            image_center_y = h/2

            dif_x = i[0] - image_center_x # dif_x = diferenca entre o centro da imagem e o centro do barco
            dif_y = i[1] - image_center_y

            meters_y = dif_x * one_pixel_in_meters  # Conversao da diferenca em pixels para metros
            meters_x = dif_y * one_pixel_in_meters

            cord_base_x = cord_x - meters_x #Coordenada da base movel
            cord_base_y = cord_y - meters_y

            for j in range(self.bases_visited  - 1):
                if abs(self.bases_stored[j].cordenates[0] - cord_base_x) <= TOL:
                    if abs(self.bases_stored[j].cordenates[1] - cord_base_y) <= TOL:
                        self.ja_mapeada = True
            if self.ja_mapeada == False:
                print("Base localizada: " + str(cord_base_x) + " , " + str(cord_base_y))
                self.mav2.go_to_local(cord_base_x, cord_base_y, 3)
                self.precision_land()
                if self.bases_visited == 0:
                    self.bases_stored.append[base("A")]
                elif self.bases_visited == 1:
                    self.bases_stored.append[base("B")]
                elif self.bases_visited == 2:
                    self.bases_stored.append[base("C")]
                elif self.bases_visited == 3:
                    self.bases_stored.append[base("D")]
                elif self.bases_visited == 4:
                    self.bases_stored.append[base("E")]
                self.bases_visited += 1
                time.sleep(3)
                self.mav2.takeoff(3) #verificar se eh o quanto sobe ou em relacao ao inicio
                time.sleep(4)
                self.mav2.go_to_local(cord_x, cord_y, 3) 
            self.ja_mapeada = False

    def trajectory(self):
        self.mav2.takeoff(3)
        self.mav2.go_to_local(0, 1, 3) 

        while abs(self.mav2.drone_pose.pose.position.x - 6) > TOL :
            self.mav2.set_vel(self.vel, 0,0,0,0,0)
            cord_x = self.mav2.drone_pose.pose.position.x
            cord_y = self.mav2.drone_pose.pose.position.y
            hsv = cv2.cvtColor(self.mav2.cam,cv2.COLOR_BGR2HSV)

            #alterar para encontrar plataformas
            lower_blue = np.array([110,50,50])
            upper_blue = np.array([130,255,255])
            mask_BasePossible = cv2.inRange(hsv, lower_blue, upper_blue)

            base_circles = cv2.HoughCircles(mask_BasePossible,cv2.HOUGH_GRADIENT,1,20,
                                param1=50,param2=30,minRadius=0,maxRadius=0)

            base_circles = np.uint16(np.around(base_circles))
            for i in base_circles[0,:]:
                # draw the outer circle
                cv2.circle(mask_BasePossible,(i[0],i[1]),i[2],(0,255,0),2)
                # draw the center of the circle
                cv2.circle(mask_BasePossible,(i[0],i[1]),2,(0,0,255),3)
                cv2.waitKey(15)

                one_pixel_in_meters = 0.0168      #Foi visto empiricamente que a uma altura de 4 metros cada pixel da imagem equivale a 0,28 metros
                h, w, c = mask_BasePossible.shape
                image_center_x = w/2 
                image_center_y = h/2

                dif_x = i[0] - image_center_x # dif_x = diferenca entre o centro da imagem e o centro do barco
                dif_y = i[1] - image_center_y

                meters_y = dif_x * one_pixel_in_meters  # Conversao da diferenca em pixels para metros
                meters_x = dif_y * one_pixel_in_meters

                cord_base_x = cord_x - meters_x #Coordenada da base movel
                cord_base_y = cord_y - meters_y
                if abs(cord_base_y - self.mav2.drone_pose.pose.position.y) < TOL :
                    self.mapping()
            time.sleep(1)

        while abs(self.mav2.drone_pose.pose.position.y - 3)  > TOL :
            self.mav2.set_vel(0, self.vel,0,0,0,0)
            cord_x = self.mav2.drone_pose.pose.position.x
            cord_y = self.mav2.drone_pose.pose.position.y
            hsv = cv2.cvtColor(self.mav2.cam,cv2.COLOR_BGR2HSV)

            #alterar para encontrar plataformas
            lower_blue = np.array([110,50,50])
            upper_blue = np.array([130,255,255])
            mask_BasePossible = cv2.inRange(hsv, lower_blue, upper_blue)

            base_circles = cv2.HoughCircles(mask_BasePossible,cv2.HOUGH_GRADIENT,1,20,
                                param1=50,param2=30,minRadius=0,maxRadius=0)

            base_circles = np.uint16(np.around(base_circles))
            for i in base_circles[0,:]:
                # draw the outer circle
                cv2.circle(mask_BasePossible,(i[0],i[1]),i[2],(0,255,0),2)
                # draw the center of the circle
                cv2.circle(mask_BasePossible,(i[0],i[1]),2,(0,0,255),3)
                cv2.waitKey(15)

                one_pixel_in_meters = 0.0168      #Foi visto empiricamente que a uma altura de 4 metros cada pixel da imagem equivale a 0,28 metros
                h, w, c = mask_BasePossible.shape
                image_center_x = w/2 
                image_center_y = h/2

                dif_x = i[0] - image_center_x # dif_x = diferenca entre o centro da imagem e o centro do barco
                dif_y = i[1] - image_center_y

                meters_y = dif_x * one_pixel_in_meters  # Conversao da diferenca em pixels para metros
                meters_x = dif_y * one_pixel_in_meters

                cord_base_x = cord_x - meters_x #Coordenada da base movel
                cord_base_y = cord_y - meters_y
                if abs(cord_base_x - self.mav2.drone_pose.pose.position.x) < TOL :
                    self.mapping()
                time.sleep(1)


        while abs(self.mav2.drone_pose.pose.position.x - 0.5)  > TOL :
            self.mav2.set_vel(-self.vel, 0,0,0,0,0)
            cord_x = self.mav2.drone_pose.pose.position.x
            cord_y = self.mav2.drone_pose.pose.position.y
            hsv = cv2.cvtColor(self.mav2.cam,cv2.COLOR_BGR2HSV)

            #alterar para encontrar plataformas
            lower_blue = np.array([110,50,50])
            upper_blue = np.array([130,255,255])
            mask_BasePossible = cv2.inRange(hsv, lower_blue, upper_blue)

            base_circles = cv2.HoughCircles(mask_BasePossible,cv2.HOUGH_GRADIENT,1,20,
                                param1=50,param2=30,minRadius=0,maxRadius=0)

            base_circles = np.uint16(np.around(base_circles))
            for i in base_circles[0,:]:
                # draw the outer circle
                cv2.circle(mask_BasePossible,(i[0],i[1]),i[2],(0,255,0),2)
                # draw the center of the circle
                cv2.circle(mask_BasePossible,(i[0],i[1]),2,(0,0,255),3)
                cv2.waitKey(15)

                one_pixel_in_meters = 0.0168      #Foi visto empiricamente que a uma altura de 4 metros cada pixel da imagem equivale a 0,28 metros
                h, w, c = mask_BasePossible.shape
                image_center_x = w/2 
                image_center_y = h/2

                dif_x = i[0] - image_center_x # dif_x = diferenca entre o centro da imagem e o centro do barco
                dif_y = i[1] - image_center_y

                meters_y = dif_x * one_pixel_in_meters  # Conversao da diferenca em pixels para metros
                meters_x = dif_y * one_pixel_in_meters

                cord_base_x = cord_x - meters_x #Coordenada da base movel
                cord_base_y = cord_y - meters_y
                if abs(cord_base_y - self.mav2.drone_pose.pose.position.y) < TOL :
                    self.mapping()
            time.sleep(1)

        while abs(self.mav2.drone_pose.pose.position.y - 1) > TOL :
            self.mav2.set_vel(0, self.vel,0,0,0,0)
            cord_x = self.mav2.drone_pose.pose.position.x
            cord_y = self.mav2.drone_pose.pose.position.y
            hsv = cv2.cvtColor(self.mav2.cam,cv2.COLOR_BGR2HSV)

            lower_orange = np.array([5,50,50])
            upper_orange = np.array([15,255,255])
            mask_tube= cv2.inRange(hsv, lower_orange, upper_orange)

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
                    # compute the bounding box of the contour and use the
                    # bounding box to compute the aspect ratio
                    (self.cx_tube, self.cy_tube, self.tw, self.th) = cv2.boundingRect(approx)
                    tube_center = (self.cx_tube, self.cy_tube)
            if tube_center != (0, 0):
                self.tube(mask_tube)

        self.mav2.go_to_local(0, 0, 3)
        self.precision_land()
        


if __name__ == "__main__":
    rclpy.init(args=sys.argv)
    mav = MAV2()
    missao = fase1(mav)
    missao.trajectory()
