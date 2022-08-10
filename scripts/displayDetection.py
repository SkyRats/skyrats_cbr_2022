import cv2
import numpy as np
import easyocr
import time

class displayDetection:
    
    def __init__(self):

        self.cap = cv2.VideoCapture('/home/renato/skyrats_ws2/src/skyrats_cbr_2022/images/Video1.mp4')
        self.squares = []
        self.reader = easyocr.Reader(['pt'])
        self.gas_percentual_image = []
        self.gas_percentual = []
        self.zero_adjustment = []
    

    def find_squares(self, contours):

        for contour in contours:
            approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)

            if len(approx) == 4:
                x, y, w, h = cv2.boundingRect(approx)
                aspectRatio = float(w) / h

                if aspectRatio >= 0.95 and aspectRatio < 1.05 and cv2.contourArea(contour) > 2500:
                    self.squares.append(contour)
                    cv2.drawContours(self.image, [approx], 0, (255, 0, 0), 4)

    def crop_image(self, mask):

        x = np.where(mask > 0)[0]
        y = np.where(mask > 0)[1]
        x1 = np.min(x)
        x2 = np.max(x)
        y1 = np.min(y)
        y2 = np.max(y)

        image_copy = self.image.copy()
        cropped_image = image_copy[x1:x2, y1:y2]

        #crop the top side of the image (gas percentual)
        cropped_image1 = cropped_image[round(cropped_image.shape[0]*0.05):round(cropped_image.shape[0]/2), round(cropped_image.shape[1]*0.08):round(cropped_image.shape[1]*0.63)]

        #crop the bottom side of the image (gas percentual)
        cropped_image2 = cropped_image[round(cropped_image.shape[0]/2):round(cropped_image.shape[0]*0.95), round(cropped_image.shape[1]*0.07):round(cropped_image.shape[1]*0.63)]

        #crop the first character of the gas percentual number
        cropped_image3 = cropped_image1[0:cropped_image1.shape[0], round(cropped_image1.shape[1]*0.06):round(cropped_image1.shape[1]*0.55)]

        #crop the second character of the gas percentual number
        cropped_image4 = cropped_image1[0:cropped_image1.shape[0], round(cropped_image1.shape[1]*0.55):cropped_image1.shape[1]]

        #crop the zero adjustment number
        cropped_image5 = cropped_image2[0:cropped_image2.shape[0], round(cropped_image2.shape[1]*0.52):round(cropped_image2.shape[1]*0.99)]

    
        #crop the image that may contain 1 or not
        cropped_image6 = cropped_image2[0:round(cropped_image2.shape[0]*0.90), round(cropped_image2.shape[1]*0.40):round(cropped_image2.shape[1]*0.51)]

        #crop the image that may contain "-" or not
        cropped_image7 = cropped_image2[round(cropped_image2.shape[0]*0.35):round(cropped_image2.shape[0]*0.55), round(cropped_image2.shape[1]*0.1):round(cropped_image2.shape[1]*0.40)]

        cv2.imshow("image2", cropped_image7)
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
        return result
    
    #check if there is something in a image
    def isEmpty(self, image, tolerance):

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray, 140, 255, cv2.CHAIN_APPROX_NONE)

        total = thresh.size
        zero = total - np.count_nonzero(thresh)
        ratio = zero/total
    
        if ratio > tolerance:
            return True

        else:
            return False


    def detection_loop(self):
        i = 0

        while self.cap.isOpened():

            self.squares = []
            success, self.image = self.cap.read()
            self.image = cv2.resize(self.image, (1280,720))
            kernel = np.ones((3, 3), np.uint8)
            self.image = cv2.dilate(self.image, kernel)
            gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
            ret, thresh = cv2.threshold(gray, 200, 255, cv2.CHAIN_APPROX_NONE)

            contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            self.find_squares(contours)

            if self.squares:


                sorted_squares = sorted(self.squares, key = cv2.contourArea, reverse = True )
                mask = np.zeros(self.image.shape, np.uint8)
                cv2.drawContours(mask, [sorted_squares[0]], 0, (0,0, 255), -1, )
                #cv2.drawContours(self.image, [sorted_squares[0]], 0, (0,0, 255), -1, )
                #cv2.imshow("mask", mask)

                


                if(i < 5):
                    
                    #apply the OCR algorithm over the numbers images
                    self.crop_image(mask)
                    self.gas_percentual = []
                    self.zero_adjustment = 0
                    self.gas_percentual.append(self.OCR(self.gas_percentual_image[0]))
                    self.gas_percentual.append(self.OCR(self.gas_percentual_image[1]))
                    self.zero_adjustment = self.OCR(self.zero_adjustment_image)[0][1]

                    one = self.isEmpty(self.one_image, 0.2)
                    minus = self.isEmpty(self.minus_image, 0.2)

                    if one:
                        self.zero_adjustment = int(self.zero_adjustment) + 10

                    if minus:
                        self.zero_adjustment = int(self.zero_adjustment)*-1

                    
                    
                    self.gasPercentual = int(str(self.gas_percentual[0][0][1]) + str(self.gas_percentual[1][0][1]))
                    print("Gas Percentual: " + str(self.gasPercentual) + "%")
                    print("Zero Adjustment: " +str(self.zero_adjustment) + "%")
                    i = i + 1

            cv2.imshow("image", self.image)

            if cv2.waitKey(5) & 0xFF == 27:
                break

    def main_interface(self):
        #time.sleep(3)
        self.detection_loop()

detection = displayDetection()
detection.main_interface()
