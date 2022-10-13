import numpy as np
from pyzbar.pyzbar import decode
import cv2
import time	
from defisheye import Defisheye	

class QRDetection():

    def __init__(self):
        self.detection = False
        self.qr_data = ""
        self.qr_debug = False
        self.qr_result
        self.frame = None
        self.det_number = 0
        self.qr_x = 0
        self.qr_y = 0
        self.qr_w = 0
        self.qr_h = 0
        self.image = None

    
    def qrdetection(self, vid):
            ret, self.frame = vid.read()
            #camera parameters
            dtype = 'linear'
            format = 'fullframe'
            fov = 140
            pfov = 90
            img_out = f"./images/out/TESTE_{dtype}_{format}_{pfov}_{fov}.jpg"

            while self.detection and self.det_number<=10:
                ret, self.frame = vid.read()
                #reducing fish eye effect
                self.image = Defisheye(self.frame, dtype=dtype, format=format, fov=fov, pfov=pfov)            
                self.image = self.image.convert(img_out)
                self.frame = self.image

                if self.qr_data != "":
                    self.det_number += 1
                
                self.qr_result = decode(self.frame)

                if len(qr_result)>0:
                    print("QR Code being detected")

                    for barcode in qr_result:       # writing qr code info in frame

                        (self.qr_x, self.qr_y, self.qr_w, self.qr_h) = barcode.rect
                        cv2.rectangle(self.frame, (self.qr_x, self.qr_y), (self.qr_x + self.qr_w, self.qr_y + self.qr_h), (0, 0, 255), 2)
                        self.qr_data = barcode.data.decode("utf-8")
                        self.qr_type = barcode.type
                        print("QR Code info: ", self.qr_data)
                        cv2.putText(self.frame, str(self.qr_data), (self.qr_x, self.qr_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 4)

                if self.qr_debug:
                    cv2.imshow("Frame", self.frame)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            # cleanup
            cv2.destroyAllWindows()
            return self.qr_result # qr_result is a list with qr infos from all readings 


    def qrtest(self, cam_id): 
        cam = cv2.VideoCapture(cam_id)
        # changing camera resolution for better processing
        cam.set(cv2.CAP_PROP_FRAME_WIDTH, 620)
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.detection = True
        self.qr_debug = True
        self.qrdetection(cam)
        cam.release()     


if __name__ == "__main__"  :
    det = QRDetection()
    det.qrtest(4)
    