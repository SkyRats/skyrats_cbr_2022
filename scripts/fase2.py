#!/usr/bin/env python3
import cv2
import rospy
import numpy as np
import time
import math
from MAV_ardupilot import MAV2
from plaquinha_classe import Buzzer
from sensorDetector import sensorDetection

class fase2:
    def __init__(self, mav):
        self.mav = mav
        
        # velocidade de cruzeiro
        self.vel_cruzeiro = 0.5

        # velocidade na qual o drone percorrerá o tubo
        self.vel_tubo = 0.2

        # altura do voo em relação ao tamanho incial da base costeira
        self.altitude = 0.3

        self.coord_tubo_final = (-6.03, 0.56)
        self.coord_tubo_inicial = (-0.63, 3.11)

        self.buz = Buzzer(22)
        
    def run(self):
        self.mav.takeoff(self.altitude + 0.2)
        rospy.sleep(7)

        # vai para o inicio do tubo
        self.mav.change_auto_speed(self.vel_cruzeiro)
        self.mav.go_to_local(self.coord_tubo_inicial[0], self.coord_tubo_inicial[1], self.altitude, yaw=math.pi/2, sleep_time=10)
        rospy.loginfo("Inicio do tubo encontrado, iniciando scaneamento")

        # vai para o fim do tubo
        self.mav.change_auto_speed(self.vel_tubo)
        detecting = sensorDetection()
        sensor_count = 0
        while sensor_count < 5 and (not rospy.is_shutdown()):
            #rospy.loginfo("Altura de voo:" + str(self.altitude))
            sensor = detecting.detect_sensors()
            if sensor == "verde":
                print("Sensor verde detectado")
                sensor_count = sensor_count + 1
            if sensor == "vermelho":
                self.mav.set_vel(0,0,0)
                self.buz.ligar(3)
                print("Sensor vermelho detectado")
                sensor_count = sensor_count + 1
            self.mav.set_position(self.coord_tubo_final[0], self.coord_tubo_final[1],self.altitude,yaw=math.pi/2)

        rospy.loginfo('Fim do tubo encontrado! Voltando para a posição incial')

        self.mav.change_auto_speed(self.vel_cruzeiro)
        self.mav.go_to_local(0, 0.2, self.altitude + 0.5, yaw=math.pi/2, sleep_time=10)

        self.mav.land()


if __name__ == "__main__":
    rospy.init_node('fase2')
    mav = MAV2()
    missao = fase2(mav)
    missao.run()

