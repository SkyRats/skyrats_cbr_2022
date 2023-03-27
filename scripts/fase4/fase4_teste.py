#!/usr/bin/env python3
import numpy as np
from pickle import FALSE
import time

TOL = 0.5 #tolerancia de erro das bases 
CAM_FOV = 1.64061 #94 graus
coords = [0, 0, 0]

class CoastBase:
    def __init__(self, mav):
        self.mav = mav
        self._coordinates = [0, 0, 0]

class Base:
    def __init__(self, name, mav, x, y, z, package):
        self.mav = mav
        self._name = name
        self._coordinates = [x, y, z]
        self._package = package

    def hasCorrectPackage(self):
        if(self._package == self._name):
            return True
        else:
            return False

    def getDistanceToDrone(self):
        global coords

        deltax = self._coordinates[0] - coords[0]
        deltay = self._coordinates[1] - coords[1]
        deltaz = self._coordinates[2] - coords[2]

        return np.sqrt(np.power(deltax, 2) + np.power(deltay, 2) + np.power(deltaz, 2))

class fase4:
    def __init__(self, mav2):
        self.mav2 = mav2
        self.coastBase = CoastBase(self.mav2)
        self.bases = [Base("A", self.mav2, 0, 1, 0, "D"), Base("B", self.mav2, 2, 3, 1, "E"), Base("C", self.mav2, 6, 7, 0, "A"), Base("D", self.mav2, 8, 0, 8, "C"), Base("E", self.mav2, 4, 4, 4, "B")] #Como as bases serão transmitidas da fase1 para a fase4?
        self.pickedPackage = None
        self.destination = None

    #def precision_land(self):   #Pousa na cruz utilizando conrole proporcional
  
    def getNearestBaseWithoutCorrectPackage(self):
        nearestBase = None
        minDistance = -1

        for base in self.bases:
            if(not base.hasCorrectPackage()):
                auxDistance = base.getDistanceToDrone()

                if(minDistance == -1 or auxDistance < minDistance):
                    minDistance = auxDistance
                    nearestBase = base

        return nearestBase

    def deliverQrcode(self):
        print("deliverQrcode")

    def pickQrcode(self): 
        print("pickQrcode")

    def findQrcode(self):
        print("findQrcode")
       
    def trajectory(self):
        global coords

        print("Start!")

        nearestBase = self.getNearestBaseWithoutCorrectPackage()

        while(not nearestBase == None):
            self.destination = nearestBase

            while(not self.destination == None):
                print("Next destination: " + self.destination._name)

                coords[0] = self.destination._coordinates[0]
                coords[1] = self.destination._coordinates[1]
                coords[2] = self.destination._coordinates[2]

                self.findQrcode()
                self.pickQrcode() #Colocar a máscara
                foundPackage = self.destination._package #Obter o pacote

                if(not foundPackage == None):
                    print("Found " + foundPackage)

                    if(not self.pickedPackage == None):
                        print("Delivering " + self.pickedPackage)
                        self.deliverQrcode()
                        for i in range(len(self.bases)):
                            if (self.destination._name == self.bases[i]._name):
                                self.bases[i]._package = self.pickedPackage
                                print(self.bases[i]._name + " -> " + self.bases[i]._package)
                                
                        self.pickedPackage = foundPackage
                        print("Picking " + self.pickedPackage)
                    else:
                        self.pickedPackage = foundPackage
                        for i in range(len(self.bases)):
                            if (self.destination._name == self.bases[i]._name):
                                self.bases[i]._package = None

                    for i in range(len(self.bases)):
                        if (self.pickedPackage == self.bases[i]._name):
                            self.destination = self.bases[i]

                else:
                    if(not self.pickedPackage == None):
                        print("Delivering " + self.pickedPackage)
                        self.deliverQrcode()
                        for i in range(len(self.bases)):
                            if (self.destination._name == self.bases[i]._name):
                                self.bases[i]._package = self.pickedPackage
                                print(self.bases[i]._name + " -> " + self.bases[i]._package)

                        self.pickedPackage = None
                        
                    self.destination = None

            nearestBase = self.getNearestBaseWithoutCorrectPackage()

        print("Finished!")

        for i in range(len(self.bases)):
            print(self.bases[i]._name + " -> " + self.bases[i]._package)

        coords[0] = self.coastBase._coordinates[0]
        coords[1] = self.coastBase._coordinates[1]       
        coords[2] = self.coastBase._coordinates[2]

if __name__ == "__main__":
    mav = None
    missao = fase4(mav)
    missao.trajectory()
    
