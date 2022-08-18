#!/usr/bin/env python3
import cv2
import rclpy
import numpy as np
from pickle import FALSE
import time
import math
sys.path.insert(0,'/home/' + os.environ["USER"] + '/skyrats_ws2/src/mavbase2')
from MAV2 import MAV2

from tf_transformations import quaternion_from_euler, euler_from_quaternion

TOL = 0.5 #tolerancia de erro das bases 
CAM_FOV = 1.64061 #94 graus

class base:
    def __init__(self, name, mav2):
        self.mav = mav2
        self._name = name
        self._cordenates = [self.mav.drone_pose.pose.position.x, self.mav.drone_pose.pose.position.y, self.mav.drone_pose.pose.position.z]

class fase2:
    def __init__(self, mav2):
        self.mav2 = mav2

        # PARTE ADICIONADA PELO BERTAN, ADAPTACOES NECESSARIAS PQ NAO FOI TESTADO AINDA
        self.vel = 0.2

        # altura do tubo medida no mapeamento
        self.th = 0.5
        # verificar se eh isso mesmo

        # inicio e fim do tubo dados no mapeamento, garantir que o inicio sera aquele mais perto da base
        self.cx_inicio_tubo = 3
        self.cy_inicio_tubo = 0
        self.cx_fim_tubo = 6
        self.cy_fim_tubo = 8
        # garantir que o sistema de coordenadas adotado estah correto, transversalidade em y

        #cosseno da transversalidade do tubo
        self.cos_dist = (
            # verificar se o sistemas de coordenadas estah certo, usa-se a diferenca em x para determinar o cosseno
            (self.cx_fim_tubo - self.cx_fim_tubo) /

            # pitagoras ai
            (math.sqrt(
                (self.cx_fim_tubo - self.cx_inicio_tubo) * (self.cx_fim_tubo - self.cx_inicio_tubo)
                + (self.cy_fim_tubo - self.cy_inicio_tubo) * (self.cy_fim_tubo - self.cy_inicio_tubo)
            ))
        )

        self.sen_dist = math.sin(math.acos(self.cos_dist))

        # dado o sistema de coordenadas da paradinha aqui, se o tubo esta comecando no eixo x ou y
        self.transversalidade_invertida = False

        # se, em y ele estah acima da base terrestre, quer dizer que comeca la 
        if self.cy_inicio_tubo > 2:
            self.transversalidade_invertida = True


        # servem para mandar uma distancia correta em x e y da grade para o drone nao bater

        # altura do tubo detectada + 0.8, garantindo uma altura menor que 1 metro do tubo.
        self.altura = self.th + 0.8
        # verificar a questao de que o drone pode bater nas bases suspensas  
       


    #def precision_land(self):   
    def get_mask(hsv , lower_color , upper_color):
    lower = np.array(lower_color)
    upper = np.array(upper_color)
    
    mask = cv2.inRange(hsv , lower, upper)

    return mask

def get_square_area(img):
    min_area = 2000 #checar valor mais apropriado
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    thresh_img = cv2.threshold(gray_img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]

    cnts = cv2.findContours(thresh_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    for cnt in cnts:
        area = cv2.contourArea(cnt)
        if area > min_area:
            return area
    return 0

def getSquares (image):
    lower_green = [29, 82, 83]
    upper_green = [97, 184, 211]

    lower_red = [0, 118, 144]
    upper_red = [4, 227, 255]

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    green_mask = get_mask(hsv, lower_green, upper_green)
    red_mask = get_mask(hsv, lower_red, upper_red)

    
    green_result = cv2.bitwise_and(image , image , mask= green_mask)
    red_result = cv2.bitwise_and(image , image , mask= red_mask)
    
    #plotting
    erode_size = 5
    dilate_size = 5

    erode_kernel = np.ones((erode_size, erode_size), np.float32)
    dilate_kernel = np.ones((dilate_size, dilate_size), np.float32)
    
    green_result = cv2.dilate(green_result, dilate_kernel)
    green_result = cv2.erode(green_result, erode_kernel)

    red_result = cv2.dilate(red_result, dilate_kernel)
    red_result = cv2.erode(red_result, erode_kernel)

    

    return (get_square_area(green_result), get_square_area(red_result)), green_result, red_result 

    def trajectory(self):
        self.mav2.takeoff(self.altura)

        # testar se a transversalidade ao contrario
        if self.transversalidade_invertida and self.cx_fim_tubo - self.cx_inicio_tubo < 0:
            # so inverte o valor do seno, agora mantem a distancia certa das grades dado a transversalidade invertida
            self.sen_dist *= -1
        
        # vai para o inicio do tubo, mantendo distancia segura da grade, 0.5 m
        self.mav2.go_to_local(self.cx_inicio_tubo + self.cos_dist * 0.5, self.cy_inicio_tubo + self.sen_dist * 0.5, self.altura)
        print('Comeco do tubo encontrado! Mantendo distancia segura da grade.')

        # vai para o fim do tubo, mantendo distancia segura da grade, 0.5 m
        self.mav2.go_to_local(self.cx_fim_tubo - self.cos_dist * 0.5, self.sen_dist, self.cy_fim_tubo - self.sen_dist * 0.5, self.altura,)
        print('Fim do tubo encontrado! Mantendo distancia segura da grade.')

        # ao invés de dar go_to_local, dah para dar set_vel e voltar quando terminar de ler todos os sensores
        # velocidade decomposta com os senos e cossenos calculados anteroriormente

        # volta para a base e pouuusa
        self.mav2.go_to_local(0, 0, 3)
        self.precision_land()

        


if __name__ == "__main__":
    rclpy.init()
    mav = MAV2()
    missao = fase1(mav)
    missao.trajectory()
    
