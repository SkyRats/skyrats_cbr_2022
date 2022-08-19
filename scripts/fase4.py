#!/usr/bin/env python3
import cv2
import rclpy
import numpy as np
from pickle import FALSE
import time
sys.path.insert(0,'/home/' + os.environ["USER"] + '/skyrats_ws2/src/mavbase2')
from MAV2 import MAV2

from tf_transformations import quaternion_from_euler, euler_from_quaternion

TOL = 0.5 #tolerancia de erro das bases 
CAM_FOV = 1.64061 #94 graus

class CoastBase:
    def __init__(self, mav):
        self.mav = mav
        self._coordinates = [self.mav.drone_pose.pose.position.x, self.mav.drone_pose.pose.position.y, self.mav.drone_pose.pose.position.z]

class Base:
    def __init__(self, mav, name, x, y, z):
        self.mav = mav
        self._name = name
        self._coordinates = [x, y, z]
        self._package = "?"

    def hasCorrectPackage():
        if(self._package == self._name):
            return True
        else
            return False

    def getDistance():
        deltax = self._coordinates[0] - self.mav.drone_pose.pose.position.x
        deltay = self._coordinates[1] - self.mav.drone_pose.pose.position.y
        deltaz = self._coordinates[2] - self.mav.drone_pose.pose.position.z

        return np.sqrt(np.pow(deltax, 2) + np.pow(deltay, 2) + np.pow(deltaz, 2))

class fase4:
    def __init__(self, mav2):
        self.mav2 = mav2
        self.coastBase = CoastBase(self.mav2)
        self.bases = [Base("A", self.mav2, 0, 0, 0), Base("B", self.mav2, 0, 0, 0), base("C", self.mav2, 0, 0, 0), base("D", self.mav2, 0, 0, 0), base("E", self.mav2, 0, 0, 0)] #Como as bases serão transmitidas da fase1 para a fase4?
        self.pickedPackage = None
        self.destination = None

    #def precision_land(self):   #Pousa na cruz utilizando conrole proporcional
  
    def getNearestBaseWithoutCorrectPackage():
        nearestBase = None
        minDistance = -1

        for base in self.bases:
            if(not base.hasCorrectPackage()):
                auxDistance = base.getDistance()

                if(minDistance == -1):
                    minDistance = auxDistance
                    nearestBase = base

                else if(auxDistance < minDistance):
                    minDistance = auxDistance
                    nearestBase = base

        return nearestBase

    def deliverQrcode(self, mask_tube): 

    def pickQrcode(self, mask_tube): 
    
    def findQrcode(self):
       
    def trajectory( self):
        self.mav2.takeoff(3)
        self.destination = self.getNearestBaseWithoutCorrectPackage()

        if(self.pickedPackage == None and (not self.destination == None)):
            self.mav2.go_to_local(destination._coordinates[0], destination._coordinates[1], destination._coordinates[2])

            self.findQrcode()
            self.pickQrcode() #Colocar a máscara
            self.pickedPackage = #Obter o pacote
            for i in range(len(self.bases)):
                if (self.destination._name == self.bases[i]._name):
                    self.bases[i]._package = "?"

            for i in range(len(self.bases)):
                if (self.pickedPackage == self.bases[i]._name):
                    self.destination = self.bases[i]

            self.mav2.go_to_local(destination._coordinates[0], destination._coordinates[1], destination._coordinates[2])

            deliverQrcode() #Colocar a máscara
            self.findQrcode()

            if(found)

            self.pickQrcode() #Colocar a máscara


        else:
            print("Finished!")



if __name__ == "__main__":
    rclpy.init()
    mav = MAV2()
    missao = fase4(mav)
    missao.trajectory()
    
