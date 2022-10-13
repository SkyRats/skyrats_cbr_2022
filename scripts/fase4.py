#!/usr/bin/env python3
import cv2
import numpy as np
from mavbase2.MAV2 import MAV2
import time

TOL = 0.5 #tolerancia de erro das bases 
CAM_FOV = 1.64061 #94 graus

class Base:
    def __init__(self, name, mav, x, y, z):
        self.mav = mav
        self._name = name
        self._coordinates = [x, y, z]
        self._package = None

    def getDistanceToDrone(self):
        deltax = self._coordinates[0] - self.mav.drone_pose.pose.position.x
        deltay = self._coordinates[1] - self.mav.drone_pose.pose.position.y
        deltaz = self._coordinates[2] - self.mav.drone_pose.pose.position.z

        return np.sqrt(np.power(deltax, 2) + np.power(deltay, 2) + np.power(deltaz, 2))

class fase4:
    def __init__(self, mav2):
        self.mav2 = mav2
        self.coastbase_coords = []
        self.bases = [Base("A", self.mav2, 0, 0, 0), Base("B", self.mav2, 0, 0, 0), Base("C", self.mav2, 0, 0, 0), Base("D", self.mav2, 0, 0, 0), Base("E", self.mav2, 0, 0, 0)] #Como as bases serão transmitidas da fase1 para a fase4?
        self.destination = None


if __name__ == "__main__":

    mav = MAV2()
    missao = fase4(mav)
    missao.trajectory()
    
