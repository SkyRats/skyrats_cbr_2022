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
        self.vel_tubo = 0.15

        # altura do voo em relação ao tamanho incial da base costeira
        self.altitude = 0.5

        # coordenadas do tubo
        self.coord_tubo_final = (-0.53, 4.36)
        self.coord_tubo_inicial = (-5.67, 1.47)

        #pino do buzzer
        self.buz = Buzzer(22)
        
    def run(self):
        self.mav.change_auto_speed(0.2)
        self.mav.takeoff(self.altitude)
        rospy.sleep(7)

        # vai para o inicio do tubo
        #self.mav.change_auto_speed(self.vel_cruzeiro)
        self.mav.go_to_local(self.coord_tubo_inicial[0], self.coord_tubo_inicial[1], self.altitude, yaw=math.pi/2, sleep_time=27)
        rospy.loginfo("Inicio do tubo encontrado, iniciando scaneamento")

        # vai para o fim do tubo enquanto detecta os sensores
        #self.mav.change_auto_speed(self.vel_tubo)
        detecting = sensorDetection()
        sensor_count = 0
        sleep_time = 55
        init_time = now = time.time()
        
        # Enquanto não detectar os 5 sensores da fase e o tempo não tiver passado do limite
        while sensor_count < 5 and (not rospy.is_shutdown()) and (now - init_time < sleep_time):
            #rospy.loginfo("Altura de voo:" + str(self.altitude))
            now = time.time()

            # Detectar sensor verde
            sensor = detecting.detect_sensors()
            if sensor == "verde":
                print("Sensor verde detectado")
                sensor_count = sensor_count + 1
            
            # Detctar sensor vermelhor e ligar o buzzer por 3 segundos
            if sensor == "vermelho":
                self.mav.set_vel(0,0,0)
                self.buz.ligar(3)
                print("Sensor vermelho detectado")
                sensor_count = sensor_count + 1
            
            # Continuar indo pro final do tubo
            self.mav.set_position(self.coord_tubo_final[0], self.coord_tubo_final[1],self.altitude,yaw=math.pi/2)

        rospy.loginfo('Fim do tubo encontrado! Voltando para a posição incial')

        #self.mav.change_auto_speed(self.vel_cruzeiro)
        self.mav.go_to_local(0, 0.2, self.altitude + 0.5, yaw=math.pi/2, sleep_time=25)

        self.mav.land()


if __name__ == "__main__":
    rospy.init_node('fase2')
    mav = MAV2()
    missao = fase2(mav)
    missao.run()

