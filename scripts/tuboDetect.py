import cv2
import numpy as np
from statistics import mode
TOL = 30

 
class tuboDetection:
    
    def __init__(self):
        self.capture = cv2.VideoCapture(0)
        self.tuboDetected = False
        self.fatorDeCorrecaoWidth =  0.9487
        self.fatorDeCorrecaoHeight = 0.6834

        self.drone_x = -278
        self.drone_y = 213
        
        
        
    def get_mask(self, hsv , lower_color , upper_color):
        lower = np.array(lower_color)
        upper = np.array(upper_color) 
        mask = cv2.inRange(hsv , lower, upper)
        return mask

    def x_conversion(self,x,width):
        return  self.drone_y + ( x - width/2) * self.fatorDeCorrecaoWidth
    
    def y_conversion(self,y,height):
        return  ( -1 * self.drone_x)+ ( y - height/2) * self.fatorDeCorrecaoWidth
        

    def get_square_area(self,img):
        min_area = 15000 #checar valor mais apropriado
        min_ratio = 10

        img_width , img_height = img.shape

        contours,junk=cv2.findContours(img,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        #cv2.drawContours(frame,contours,-1,(255,0,0),3)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            #print(f"Area: {area}")
        
            if area >= min_area:
                cv2.drawContours(img,[contour],0,(255,0,0),3)
                (x,y), (w,h), theta = cv2.minAreaRect(contour)

                if h > w:
                    temp = h
                    h = w
                    w = temp

                if(w/h) > min_ratio:
                    w = round(w * self.fatorDeCorrecaoWidth,2)
                    h = round(h * self.fatorDeCorrecaoHeight,2)
                    print(f"Comprimento do tubo : {w} cm")
                    print(f"Largura do tubo : {h } cm")

                    cv2.imwrite("./tuboMask.png", img)

                    rect = ((x, y), (w, h), theta)
                    box = cv2.boxPoints(rect) 
                    print("Coordenadas do tubo:")

                    x_extremo1 =  img_width - ((box[0][1] + box[1][1])/2)
                    y_extremo1 =  img_height - ((box[0][0] + box[1][0])/2)
                    x_extremo2 =  img_width - ((box[2][1] + box[3][1])/2)
                    y_extremo2 =  img_height - ((box[2][0] + box[3][0])/2)
                    print(f"Point 1  x:{self.x_conversion(x_extremo1,img_width)}  y:{self.y_conversion(y_extremo1, img_height)}")
                    print(f"Point 2  x:{self.x_conversion(x_extremo2,img_width)}  y:{self.y_conversion(y_extremo2, img_height)}")

                    # box = np.int0(box)
                    # cv2.drawContours(self.frame,[box],0,(0,0,255),2)
                    self.tuboDetected = True
        
        return None
        


    def getSquares (self, image):
        

        #orange mask
        lower_orange =   [ 0, 66, 193]
        upper_orange =  [ 38, 231, 255]

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

    while  not detecting.tuboDetected:
        detecting.detect_sensors()

          
    detecting.capture.release()
    cv2.destroyAllWindows()

    cv2.waitKey(0)
    
