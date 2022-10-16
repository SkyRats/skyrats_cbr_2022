from email.mime import image
import cv2
import numpy as np
import easyocr
import time
from statistics import mode
from defisheye import Defisheye
from imutils.perspective import four_point_transform

class displayDetection:
    
    def __init__(self):

        self.cap = cv2.VideoCapture(2)

        self.squares = []
        self.gas_percentual_list1 = []
        self.gas_percentual_list2=[]
        self.zero_adjustment_list = []
        self.reader = easyocr.Reader(['pt'])
        self.gas_percentual_image = []
        self.gas_percentual = []
        self.zero_adjustment = []
    

    def find_potentials(self, contours):
        
        shapes = []

        for contour in contours:
            approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)
            
            if len(approx) == 4 and cv2.arcLength(contour,True) > 200:
                shapes.append(approx)
        
        return shapes
    
    def find_squares(self, contours):

        self.squares= []


        for contour in contours:
            approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)

            if len(approx) == 4:
                x, y, w, h = cv2.boundingRect(approx)
                aspectRatio = float(w) / h


                if aspectRatio >= 0.95 and aspectRatio < 1.1 and cv2.contourArea(contour) > 3000:

                    self.reshape = True
                    self.reshaped_image = four_point_transform(self.image, approx.reshape(4,2)) 
                    #cv2.imshow("reshape", self.reshaped_image)
                    #self.squares.append(contour)
                    #cv2.drawContours(self.image, [approx], 0, (255, 0, 0), 4)
                    #cv2.rectangle(self.image,(x,y),(x+y, y+h), (0,255,0),2)

    def find_reshaped_square(self):

        gray = cv2.cvtColor(self.reshaped_image, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray, 140, 255, cv2.CHAIN_APPROX_NONE)
        contours, hierarchy = cv2.findContours(gray, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)

            if len(approx) == 4:
                x, y, w, h = cv2.boundingRect(approx)
                aspectRatio = float(w) / h


                if aspectRatio >= 0.95 and aspectRatio < 1.1 and cv2.contourArea(contour) > 3000:

                    self.squares.append(contour)
                    cv2.drawContours(self.reshaped_image, [approx], 0, (255, 0, 0), 4)

                


    def crop_image(self):

        x = np.where(self.reshaped_image > 0)[0]
        y = np.where(self.reshaped_image > 0)[1]
        x1 = np.min(x)
        x2 = np.max(x)
        y1 = np.min(y)
        y2 = np.max(y)

        image_copy = self.reshaped_image.copy()
        cropped_image = image_copy[x1:x2, y1:y2]
        
        #crop the top side of the image (gas percentual)
        cropped_image1 = cropped_image[round(cropped_image.shape[0]*0.05):round(cropped_image.shape[0]/2), round(cropped_image.shape[1]*0.08):round(cropped_image.shape[1]*0.63)]
        cv2.imshow("crop", cropped_image1)
        #crop the bottom side of the image (gas percentual)
        cropped_image2 = cropped_image[round(cropped_image.shape[0]/2):round(cropped_image.shape[0]*0.95), round(cropped_image.shape[1]*0.07):round(cropped_image.shape[1]*0.63)]

        #crop the first character of the gas percentual number
        cropped_image3 = cropped_image1[0:cropped_image1.shape[0], round(cropped_image1.shape[1]*0.06):round(cropped_image1.shape[1]*0.55)]
        cv2.imshow("crop2", cropped_image3)
        #crop the second character of the gas percentual number
        cropped_image4 = cropped_image1[0:cropped_image1.shape[0], round(cropped_image1.shape[1]*0.55):cropped_image1.shape[1]]
        
        #crop the zero adjustment number
        cropped_image5 = cropped_image2[0:cropped_image2.shape[0], round(cropped_image2.shape[1]*0.52):round(cropped_image2.shape[1]*0.99)]

    
        #crop the image that may contain 1 or not
        cropped_image6 = cropped_image2[0:round(cropped_image2.shape[0]*0.90), round(cropped_image2.shape[1]*0.40):round(cropped_image2.shape[1]*0.51)]

        #crop the image that may contain "-" or not
        cropped_image7 = cropped_image2[round(cropped_image2.shape[0]*0.43):round(cropped_image2.shape[0]*0.57), round(cropped_image2.shape[1]*0.055):round(cropped_image2.shape[1]*0.36)]

        #cv2.imshow("minus", cropped_image7)
        #cv2.imshow("one", cropped_image6)
        
        #append the numbers images in their lists

        self.gas_percentual_image = []

        self.gas_percentual_image.append(cropped_image3)
        self.gas_percentual_image.append(cropped_image4)
        self.zero_adjustment_image = cropped_image5
        self.one_image = cropped_image6
        self.minus_image = cropped_image7

        
    #Optical Character Recognition
    def OCR(self, image):

        result = self.reader.readtext(image)
        print(result)
        # print(number)

        if result:

            content = result[0][1]
            accuracy = result[0][2]

            if accuracy > 0.85 and self.checkInt(content):
                
                return [content, accuracy]

            else: 

                return [False, False]

        else:

            return [False, False]

    def checkInt(self,str):
        try:
            int(str)
            return True
        except ValueError:
            return False

    #check if there is something in a image

    
    def isEmpty(self, image, tolerance):

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        ret, thresh = cv2.threshold(gray, 140, 255, cv2.CHAIN_APPROX_NONE)
        # cv2.imshow("thresh", thresh)

        total = thresh.size
        zero = total - np.count_nonzero(thresh)
        ratio = zero/total
        print(ratio)
    
        if ratio > tolerance:
            return True

        else:
            return False


    def detection_loop(self):
        i = 0
        self.reshape = False
        while self.cap.isOpened() and i<200 :

            self.squares = []
            success, self.image = self.cap.read()
            dtype = 'linear'
            format = 'fullframe'
            fov = 140
            pfov = 90

            
            img_out = f"./images/out/TESTE_{dtype}_{format}_{pfov}_{fov}.jpg"

            self.image = Defisheye(self.image, dtype=dtype, format=format, fov=fov, pfov=pfov)
            # obj.convert(img_out)
            
            self.image = self.image.convert(img_out)
            # self.image = cv2.resize(self.image, (1280,720))
            kernel = np.ones((3, 3), np.uint8)
            self.image = cv2.dilate(self.image, kernel)
            # self.image = cv2.erode(self.image, kernel)
            #canny = cv2.Canny(self.image,100,200)
            #cv2.imshow("canny", canny)
            gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
            # blurred = cv2.GaussianBlur(gray, (7, 7), 0)
            thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 2)
            # ret, thresh = cv2.threshold(gray, 140, 255, cv2.CHAIN_APPROX_NONE)
            cv2.imshow("thresh", thresh)

            #cv2.imshow("thresh", thresh)

            contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            self.find_squares(contours)

            if self.reshape:
                self.find_reshaped_square()

            if self.squares:


                sorted_squares = sorted(self.squares, key = cv2.contourArea, reverse = True )
                #mask = np.zeros(self.image.shape, np.uint8)
                #cv2.drawContours(mask, [sorted_squares[0]], 0, (0,0, 255), -1, )
                #cv2.drawContours(self.image, [sorted_squares[0]], 0, (0,0, 255), -1, )
                #cv2.imshow("mask", mask)

                self.crop_image()


                if(i < 20):

                    
                    
                    #apply the OCR algorithm over the numbers images
                    
                    self.gas_percentual = [0,0]
                    self.zero_adjustment = 0

                    self.gas_percentual = [self.OCR(self.gas_percentual_image[0])[0], self.OCR(self.gas_percentual_image[1])[0]]                 
                    self.zero_adjustment = self.OCR(self.zero_adjustment_image)[0]

                    one = self.isEmpty(self.one_image, 0.2)
                    minus = self.isEmpty(self.minus_image, 0.2)


                    if self.gas_percentual[0]:
                        self.gas_percentual_list1.append(self.gas_percentual[0])
                    
                    else:

                        oneGasPercentual0 = self.isEmpty(self.gas_percentual_image[0], 0.13)
                        if len(self.squares)==(i+1) and oneGasPercentual0 :
                            self.gas_percentual_list1.append(1)


                    if self.gas_percentual[1]:
                                               
                        # self.gasPercentual = int(str(self.gas_percentual[0]) + str(self.gas_percentual[1]))
                        self.gas_percentual_list2.append(self.gas_percentual[1])
                        # print("Gas Percentual: " + str(self.gasPercentual) + "%")
                    
                    # if self.squares and self.gas_percentual[0]==0 and self.gas_percentual_list1==[]:
                    #     self.gas_percentual_list1.append(1)

                    # if self.squares and self.gas_percentual[1]==0 and self.gas_percentual_list2==[]:
                    #     self.gas_percentual_list2.append(1)

                    else:
                        oneGasPercentual1 = self.isEmpty(self.gas_percentual_image[1], 0.13)
                        if len(self.squares)==(i+1) and oneGasPercentual1:
                            self.gas_percentual_list2.append(1)
                            print("oi")
                

                    if self.zero_adjustment:

                        if one:
                            self.zero_adjustment = int(self.zero_adjustment) + 10

                        if minus:
                            self.zero_adjustment = int(self.zero_adjustment)*-1

                        self.zero_adjustment_list.append(self.zero_adjustment)

                        #print("Zero Adjustment: " +str(self.zero_adjustment) + "%")

                    else: 
                        oneZeroAdj = self.isEmpty(self.zero_adjustment_image, 0.12)
                        if len(self.squares)==(i+1) and oneZeroAdj:

                            print("Li nada")

                            self.zero_adjustment =1;

                            if one:
                                self.zero_adjustment = int(self.zero_adjustment) + 10

                            if minus:
                                self.zero_adjustment = int(self.zero_adjustment)*-1

                            self.zero_adjustment_list.append(self.zero_adjustment)

                        
                    
                    i = i + 1

                else:

                    if(len(self.gas_percentual_list1)!=0 and len(self.gas_percentual_list2)!=0):

                        self.gas_percentual[0]=mode(self.gas_percentual_list1)
                        self.gas_percentual[1]=mode(self.gas_percentual_list2)

                        self.gasPercentual = int(str(self.gas_percentual[0]) + str(self.gas_percentual[1]))
                        print("Gas Percentual: " + str(self.gasPercentual) + "%")

                    if(len(self.zero_adjustment_list)!=0):

                        self.zero_adjustment=mode(self.zero_adjustment_list)
                        print("Zero Adjustment: " +str(self.zero_adjustment) + "%")

                    return

               

            cv2.imshow("image", self.image)

            if cv2.waitKey(5) & 0xFF == 27:
                break

    def main_interface(self):
        #time.sleep(3)
        self.detection_loop()

detection = displayDetection()
detection.main_interface()
