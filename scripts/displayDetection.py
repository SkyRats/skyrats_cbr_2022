import cv2
import numpy as np
import easyocr
import time

class displayDetection:
    
    def __init__(self):

        self.cap = cv2.VideoCapture(0)
        self.squares = []
        self.reader = easyocr.Reader(['pt'])
        self.gas_percentual_image = []
        self.gas_percentual = []

    def find_squares(self, contours):

        for contour in contours:
            approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)

            if len(approx) == 4:
                x, y, w, h = cv2.boundingRect(approx)
                aspectRatio = float(w) / h

                if aspectRatio >= 0.95 and aspectRatio < 1.05 and cv2.contourArea(contour) > 600:
                    self.squares.append(contour)
                    #cv2.drawContours(self.image, [approx], 0, (255, 0, 0), 4)

    def crop_image(self, mask):

        x = np.where(mask > 0)[0]
        y = np.where(mask > 0)[1]
        x1 = np.min(x)
        x2 = np.max(x)
        y1 = np.min(y)
        y2 = np.max(y)

        image_copy = self.image.copy()
        cropped_image = image_copy[x1:x2, y1:y2]
        cropped_image1 = cropped_image[round(cropped_image.shape[0] * 0.03):round(cropped_image.shape[0] / 2),
                         round(cropped_image.shape[1] * 0.01):round(cropped_image.shape[1] * 0.65)]

        cropped_image2 = cropped_image[round(cropped_image.shape[0] / 2):round(cropped_image.shape[0]),
                         round(cropped_image.shape[1] * 0.07):round(cropped_image.shape[1] * 0.65)]

        self.gas_percentual_image = []

        #crop the first character of the gas percentual number
        cropped_image3 = cropped_image1[0:cropped_image1.shape[0], round(cropped_image1.shape[1]*0.10):round(cropped_image1.shape[1]*0.55)]

        #crop the second character of the gas percentual number
        cropped_image4 = cropped_image1[0:cropped_image1.shape[0], round(cropped_image1.shape[1]*0.55):cropped_image1.shape[1]]

        #crop the zero adjustment number
        cropped_image5 = cropped_image2[0:cropped_image2.shape[0], round(cropped_image2.shape[1]*0.52):round(cropped_image2.shape[1]*0.99)]
        
        self.gas_percentual_image.append(cropped_image3)
        self.gas_percentual_image.append(cropped_image4)
       
        

    def OCR(self, image):

        result = self.reader.readtext(image)
        return result
        
    def detection_loop(self):
        i = 0

        while self.cap.isOpened():
            self.squares = []
            success, self.image = self.cap.read()

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
                cv2.drawContours(self.image, [sorted_squares[0]], 0, (0,0, 255), -1, )
                cv2.imshow("mask", mask)

                self.crop_image(mask)


                if(i < 10):

                    self.gas_percentual = []
                    self.gas_percentual.append(self.OCR(self.gas_percentual_image[0]))
                    self.gas_percentual.append(self.OCR(self.gas_percentual_image[1]))
                    

                    print(self.gas_percentual[0])
                    print(self.gas_percentual[1])
                    
                    i = i + 1

            cv2.imshow("image", self.image)

            if cv2.waitKey(5) & 0xFF == 27:
                break

    def main_interface(self):
        #time.sleep(3)
        self.detection_loop()

detection = displayDetection()
detection.main_interface()
