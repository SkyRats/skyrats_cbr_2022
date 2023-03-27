import cv2
import numpy as np
from statistics import mode

# Tolerância de detecção de falha
# Quanto maior, mais tempo levará para detectar um sensor pela primeira vez
# Mas ela continuará detectando ele mesmo com períodos maiores de falha
TOL = 15


class sensorDetection:
    
    def __init__(self):
        self.capture = cv2.VideoCapture(0)
        self.redSquaresCount = 0
        self.greenSquaresCount = 0
        self.greenRecord = [0 for i in range(TOL)]
        self.redRecord =[0 for i in range(TOL)]
        
        
    def get_mask(self, hsv , lower_color , upper_color):
        # Monta a mascara com os ranges selecionados
        lower = np.array(lower_color)
        upper = np.array(upper_color) 
        mask = cv2.inRange(hsv , lower, upper)
        return mask

    def get_square_area(self,img):
        # Retorna uma lista com os quadrados que tem area maior ou igual a min_area

        squaresDetected = []
        min_area = 300 # Area minima para um quadrado ser contabilizado
    
        contours,junk=cv2.findContours(img,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        # Se quiser ver os contornos:
        # cv2.drawContours(frame,contours,-1,(255,0,0),3)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area >= min_area:
                squaresDetected.append(contour)

                # Desenha um retangulo em torno do quadrado detectado (opcional)
                x,y,w,h=cv2.boundingRect(contour)
                cv2.rectangle(self.frame,(x,y),(x+w,y+h),(0,0,255),3)

        return squaresDetected

    def getSquares (self, image):
        
        # Mascaras verde e vermelha
        # Calibrar antes de usar
        lower_green = [54, 78, 91]
        upper_green = [90, 193, 216]

        lower_red = [0, 143, 0]
        upper_red = [6, 228, 255]

        # Criando as máscaras
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        green_mask = self.get_mask(hsv, lower_green, upper_green)
        red_mask = self.get_mask(hsv, lower_red, upper_red)


        # Os passos abaixo servem só para melhorar a visualizacao
        # da mascara para o usuario, nao interferem na deteccao
        green_result = cv2.bitwise_and(image , image , mask= green_mask)
        red_result = cv2.bitwise_and(image , image , mask= red_mask)

        erode_size = 5
        dilate_size = 5

        erode_kernel = np.ones((erode_size, erode_size), np.float32)
        dilate_kernel = np.ones((dilate_size, dilate_size), np.float32)
        
        green_result = cv2.dilate(green_result, dilate_kernel)
        green_result = cv2.erode(green_result, erode_kernel)

        red_result = cv2.dilate(red_result, dilate_kernel)
        red_result = cv2.erode(red_result, erode_kernel)

        # Retorna a contagem de quadrados verdes e vermelhos e as imagens para visualizacao das mascaras
        return (self.get_square_area(green_mask), self.get_square_area(red_mask)), green_result, red_result
    
    def detect_sensors(self):
        sensor = 0
        
        success, self.frame = self.capture.read()
        if success == False:
            raise ConnectionError

        (green_detected,red_detected), green_image, red_image = self.getSquares(self.frame)

        # Para ver as imagens das mascaras aplicadas e da camera usada
        # cv2.imshow('Red', red_image)
        # cv2.imshow('Green', green_image)
        # cv2.imshow('Camera',self.frame)

        # Updating Records
        # Retira o primeiro elemento e adiciona o novo no final
        self.greenRecord.pop(0)
        self.greenRecord.append(len(green_detected))

        self.redRecord.pop(0)
        self.redRecord.append(len(red_detected))

        # Checa se a moda da lista de sensores mudou, indicando
        # que um novo sensor foi detectado ou que um sensor detectado ja saiu da tela

        # Sensor saiu da tela
        if(mode(self.greenRecord) < self.greenSquaresCount):
            self.greenSquaresCount = mode(self.greenRecord)
        # Novo sensor entrou na tela
        if(mode(self.greenRecord) > self.greenSquaresCount):
            self.greenSquaresCount = mode(self.greenRecord)
            self.greenRecord = [ self.greenSquaresCount for i in range(TOL)]
            sensor = "verde"

        # Sensor saiu da tela
        if(mode(self.redRecord) < self.redSquaresCount):
            self.redSquaresCount = mode(self.redRecord)
        # Novo sensor entrou na tela
        if(mode(self.redRecord) > self.redSquaresCount):
            self.redSquaresCount = mode(self.redRecord)
            self.redRecord = [ self.redSquaresCount for i in range(TOL)]
            sensor = "vermelho"

        return sensor


# Enquanto não chegar ao fim do tubo, ir lendo os sensores da tela
if __name__ == "__main__":
    detecting = sensorDetection()
    fimdoTubo = False

    while not fimdoTubo :
        sensor = detecting.detect_sensors()
        if sensor == "verde":
            print("Sensor verde detectado")
        if sensor == "vermelho":
            print("Sensor vermelho detectado")
            
            
    detecting.capture.release()
    cv2.destroyAllWindows()

    cv2.waitKey(0)
    
