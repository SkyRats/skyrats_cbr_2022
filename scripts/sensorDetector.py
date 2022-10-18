import cv2
import numpy as np
from statistics import mode
from plaquinha_classe import Buzzer

class sensorDetection:
    
    def __init__(self):
        self.capture = cv2.VideoCapture(0)
        self.redSquaresCount = 0
        self.greenSquaresCount = 0
        self.greenRecord = [0 for i in range(10)]
        self.redRecord =[0 for i in range(10)]
        
        
    def get_mask(self, hsv , lower_color , upper_color):
        lower = np.array(lower_color)
        upper = np.array(upper_color) 
        mask = cv2.inRange(hsv , lower, upper)
        return mask

    def get_square_area(self,img):
        squaresDetected = []
        min_area = 400 #checar valor mais apropriado
    
        contours,junk=cv2.findContours(img,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        #cv2.drawContours(frame,contours,-1,(255,0,0),3)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area >= min_area:
                #cv2.drawContours(frame,[contour],0,(255,0,0),3)
                x,y,w,h=cv2.boundingRect(contour)
                cv2.rectangle(self.frame,(x,y),(x+w,y+h),(0,0,255),3)
                squaresDetected.append(contour)
        
        

        return squaresDetected

    def getSquares (self, image):
        
        #Mascaras testes
        lower_green = [10, 113, 86]
        upper_green = [124, 239, 179]

        lower_red = [0, 148, 160]
        upper_red = [39, 255, 187]

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        green_mask = self.get_mask(hsv, lower_green, upper_green)
        red_mask = self.get_mask(hsv, lower_red, upper_red)


        #green_result é a imagem colorida da máscara
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


        return (self.get_square_area(green_mask), self.get_square_area(red_mask)), green_result, red_result
    
    def detect_sensors(self):
        sensor = 0
        
        success, self.frame = self.capture.read()
        if success == False:
            raise ConnectionError

        (green_detected,red_detected), green_image, red_image = self.getSquares(self.frame)

        cv2.imshow('Red', red_image)
        cv2.imshow('Green', green_image)
        cv2.imshow('Camera',self.frame)

        #Updating Records
        self.greenRecord.pop(0)
        self.greenRecord.append(len(green_detected))

        self.redRecord.pop(0)
        self.redRecord.append(len(red_detected))


        if(mode(self.greenRecord) < self.greenSquaresCount):
            self.greenSquaresCount = mode(self.greenRecord)
        if(mode(self.greenRecord) > self.greenSquaresCount):
            self.greenSquaresCount = mode(self.greenRecord)
            self.greenRecord = [ self.greenSquaresCount for i in range(10)]
            sensor = "verde"

        if(mode(self.redRecord) < self.redSquaresCount):
            self.redSquaresCount = mode(self.redRecord)
        if(mode(self.redRecord) > self.redSquaresCount):
            self.redSquaresCount = mode(self.redRecord)
            self.redRecord = [ self.redSquaresCount for i in range(10)]
            sensor = "vermelho"
    
        if cv2.waitKey(20) & 0xFF == ord('q'):
            return 1

        return sensor



if __name__ == "__main__":
    detecting = sensorDetection()
    sensorCount = 0
    fimdoTubo = False
    buz = Buzzer(22)

    while not fimdoTubo :
        sensor = detecting.detect_sensors()
        if sensor == "verde":
            print("Sensor verde detectado")
            #Ações quando o verde for detectado
        if sensor == "vermelho":
            buz.ligar(3)
            print("Sensor vermelho detectado")
            #Ações quando o vermelho for detectado
            
    detecting.capture.release()
    cv2.destroyAllWindows()

    cv2.waitKey(0)
    
