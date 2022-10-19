from xml.dom import NotFoundErr
import cv2
import numpy as np
from statistics import mode
TOL = 30

 
#VERIFICAR AREA MINNIMA 
#VERIFICAR FATOR DE CORREÇÃO

class tuboDetection:
    
    def __init__(self):
        self.capture = cv2.VideoCapture(0)
        self.redSquaresCount = 0
        self.orangeSquaresCount = 0
        self.orangeRecord = [0 for i in range(TOL)]
        self.tuboDetected = False
        self.fatorDeCorrecaoWidth = 1.29
        self.fatorDeCorrecaoHeight = 0.714
        
        
    def get_mask(self, hsv , lower_color , upper_color):
        lower = np.array(lower_color)
        upper = np.array(upper_color) 
        mask = cv2.inRange(hsv , lower, upper)
        return mask

    def get_square_area(self,img):
        squaresDetected = []
        min_area = 1000 #checar valor mais apropriado
    
        contours,junk=cv2.findContours(img,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        #cv2.drawContours(frame,contours,-1,(255,0,0),3)
        
        for contour in contours:
            area = cv2.contourArea(contour)
        
            if area >= min_area:
                #cv2.drawContours(frame,[contour],0,(255,0,0),3)
                (x,y), (w,h), theta = cv2.minAreaRect(contour)

                #cv2.rectangle(self.frame,(x,y),(x+w,y+h),(0,0,255),3)
                if h > w:
                    temp = h
                    h = w
                    w = temp

                w = round(w * self.fatorDeCorrecaoWidth,2)
                h = round(h * self.fatorDeCorrecaoHeight,2)
                print(f"Comprimento do tubo : {w} cm")
                print(f"Largura do tubo : {h } cm")
                cv2.imwrite("./tuboMask.png", img)
                self.tuboDetected = True
        
        return None
        


    def getSquares (self, image):
        

        #orange mask
        lower_orange =  [ 0, 122, 148]
        upper_orange = [ 51, 250, 255]

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        orange_mask = self.get_mask(hsv, lower_orange, upper_orange)


        #orange_result é a imagem colorida da máscara
        orange_result = cv2.bitwise_and(image , image , mask= orange_mask)

        #plotting
        erode_size = 5
        dilate_size = 5

        erode_kernel = np.ones((erode_size, erode_size), np.float32)
        dilate_kernel = np.ones((dilate_size, dilate_size), np.float32)
        
        orange_result = cv2.dilate(orange_result, dilate_kernel)
        orange_result = cv2.erode(orange_result, erode_kernel)


        return self.get_square_area(orange_mask), orange_result
    
    def detect_sensors(self):
        sensor = 0
        
        
        success, self.frame = self.capture.read()
        if success == False:
            raise ConnectionError

        medidas_tubo , orange_image = self.getSquares(self.frame)
        
        
        cv2.imshow('orange', orange_image)
        cv2.imshow('Camera',self.frame)

        
    
        if cv2.waitKey(20) & 0xFF == ord('q'):
            return 1





if __name__ == "__main__":
    detecting = tuboDetection()
    fimdoTubo = False
   # buz = Buzzer(22)

    while  not detecting.tuboDetected :
        detecting.detect_sensors()
        
    #tubo medidas = [x,y,h,w]

            
    detecting.capture.release()
    cv2.destroyAllWindows()

    cv2.waitKey(0)
    
    
