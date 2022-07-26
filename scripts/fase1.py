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

        self.bases_visitadas = 0
        self.base_econtrada
        self.bases_salvas = []   
        self.ja_mapeada = False  
        self.vel = 0.2

    #def precision_land(self):   #Pousa na cruz utilizando conrole proporcional
  
    #def tubo(self):          
    
    def mapping(self):
        cord_x = self.mav2.drone_pose.pose.position.x
        cord_y = self.mav2.drone_pose.pose.position.y
        self.time(8)    
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
            image_center_x = self.image_pixel_width/2   #Mudar para variavel
            image_center_y = self.image_pixel_height/2

            dif_x = i[0] - image_center_x # dif_x = diferenca entre o centro da imagem e o centro do barco
            dif_y = i[1] - image_center_y

            meters_y = dif_x * one_pixel_in_meters  # Conversao da diferenca em pixels para metros
            meters_x = dif_y * one_pixel_in_meters

            cord_base_x = cord_x - meters_x #Coordenada da base movel
            cord_base_y = cord_y - meters_y

            for j in range(self.bases_visitadas  - 1):
                if abs(self.bases_salvas[j].cordenates[0] - cord_base_x) <= TOL:
                    if abs(self.bases_salvas[j].cordenates[1] - cord_base_y) <= TOL:
                        self.ja_mapeada = True
            if self.ja_mapeada == False:
                print("Base localizada: " + str(cord_base_x) + " , " + str(cord_base_y))
                self.mav2.go_to_local(cord_base_x, cord_base_y, 3)
                self.precision_land()
                if self.bases_visitadas == 0:
                    self.bases_salvas.append[base("A")]
                elif self.bases_visitadas == 1:
                    self.bases_salvas.append[base("B")]
                elif self.bases_visitadas == 2:
                    self.bases_salvas.append[base("C")]
                elif self.bases_visitadas == 3:
                    self.bases_salvas.append[base("D")]
                elif self.bases_visitadas == 4:
                    self.bases_salvas.append[base("E")]
                self.bases_visitadas += 1
                self.time(8)
                self.mav2.takeoff(3) #verificar se eh o quanto sobe ou em relacao ao inicio
                self.time(4)
                self.mav2.go_to_local(cord_x, cord_y, 3) 
            self.ja_mapeada = False

    def trajectory(self):
        self.mav2.takeoff(3)
        self.mav2.go_to_local(0, 1, 3) 

        trg_x = abs(self.mav2.drone_pose.pose.position.x - 6)
        while trg_x > TOL :
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
                image_center_x = self.image_pixel_width/2   
                image_center_y = self.image_pixel_height/2

                dif_x = i[0] - image_center_x # dif_x = diferenca entre o centro da imagem e o centro do barco
                dif_y = i[1] - image_center_y

                meters_y = dif_x * one_pixel_in_meters  # Conversao da diferenca em pixels para metros
                meters_x = dif_y * one_pixel_in_meters

                cord_base_x = cord_x - meters_x #Coordenada da base movel
                cord_base_y = cord_y - meters_y
                if abs(cord_base_y - self.mav2.drone_pose.pose.position.y) < TOL :
                    self.mapping()
            self.time(8)

        trg_y = abs(self.mav2.drone_pose.pose.position.y - 3)
        while trg_y > TOL :
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
                image_center_x = self.image_pixel_width/2   
                image_center_y = self.image_pixel_height/2

                dif_x = i[0] - image_center_x # dif_x = diferenca entre o centro da imagem e o centro do barco
                dif_y = i[1] - image_center_y

                meters_y = dif_x * one_pixel_in_meters  # Conversao da diferenca em pixels para metros
                meters_x = dif_y * one_pixel_in_meters

                cord_base_x = cord_x - meters_x #Coordenada da base movel
                cord_base_y = cord_y - meters_y
                if abs(cord_base_x - self.mav2.drone_pose.pose.position.x) < TOL :
                    self.mapping()
            self.time(8)

        trg_x = abs(self.mav2.drone_pose.pose.position.x - 0.5)
        while trg_x > TOL :
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
                image_center_x = self.image_pixel_width/2   
                image_center_y = self.image_pixel_height/2

                dif_x = i[0] - image_center_x # dif_x = diferenca entre o centro da imagem e o centro do barco
                dif_y = i[1] - image_center_y

                meters_y = dif_x * one_pixel_in_meters  # Conversao da diferenca em pixels para metros
                meters_x = dif_y * one_pixel_in_meters

                cord_base_x = cord_x - meters_x #Coordenada da base movel
                cord_base_y = cord_y - meters_y
                if abs(cord_base_y - self.mav2.drone_pose.pose.position.y) < TOL :
                    self.mapping()
            self.time(8)

        self.mav2.go_to_local(0, 0, 3)
        self.precision_land()

    #def time(self, t):  #Funcao criada para dar um delay no codigo por N segundos


if __name__ == "__main__":
    rclpy.init(args=sys.argv)
    mav = MAV2()
    missao = fase1(mav)
    missao.trajectory()
