import numpy as np
from pyzbar.pyzbar import decode
import cv2
import time	
from defisheye import Defisheye	

class QRDetection():

    def __init__(self):
        self.detected = False
        self.qr_data = ""
        self.qr_debug = False
        self.qr_result =None
        self.cam = None # self.cam = cv2.VideoCapture(cam_id)
        self.frame = None # self.frame é uma imagem de cv (numpy.ndarray) que será processada nas funções
        self.det_number = 0
        self.qrs = []
        self.fisheye = False
        self.bases_detected = 0
        self.cam_id = None
    
    def qrdetection(self, vid):
        self.frame = vid.read()
        
        #camera parameters
        dtype = 'linear'
        format = 'fullframe'
        fov = 140
        pfov = 90
        img_out = f"./images/out/TESTE_{dtype}_{format}_{pfov}_{fov}.jpg"
        timeout = 5 # timeout in seconds for the drone to give up the qr detection
        init = time.time()
        now = time.time()

        while not self.detected or self.det_number < 2:
            self.qr_data = ""
            now = time.time()
            if(now - init) > timeout:
                return "Timeout for QR detection exceeded"
            ret, self.frame = vid.read()
            #reducing fish eye effect
            self.frame = Defisheye(self.frame, dtype=dtype, format=format, fov=fov, pfov=pfov)            
            self.frame = self.frame.convert(img_out)
            
            self.qr_result = decode(self.frame)

            if len(self.qr_result)>0:
                self.detected = True
                init = time.time()
                self.det_number += 1

                for barcode in self.qr_result:       # writing qr code info in frame

                    (qr_x, qr_y, qr_w, qr_h) = barcode.rect
                    cv2.rectangle(self.frame, (qr_x, qr_y), (qr_x + qr_w, qr_y + qr_h), (0, 0, 255), 2)
                    self.qr_data = barcode.data.decode("utf-8")
                    if self.qr_data not in self.qrs:
                        self.qrs.append(self.qr_data)
                    cv2.putText(self.frame, str(self.qr_data), (qr_x, qr_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 4)
            cv2.putText(self.frame, "Number of detections: " + str(self.det_number), (25, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 4)

            if self.qr_debug:
                cv2.imshow("Frame", self.frame)
                ret, self.frame = self.cam.read()
                print("QR data: " + str(self.qr_data))
                self.detected = False
                init = time.time()
                

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
                #cv2.destroyAllWindows()

        # cleanup
        cv2.destroyAllWindows()
        return self.qr_data # qr_result is a list with qr infos from all readings 


    def qrtest(self, cam_id=None, frame=None): 
        if cam_id != None:
            self.cam_id = cam_id
            self.cam = cv2.VideoCapture(cam_id)
        else: 
            self.cam = frame
        # changing camera resolution for better processing
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.qr_debug = True
        result = self.qrdetection(self.cam)
        self.cam.release()    
        return result


if __name__ == "__main__"  :
    det = QRDetection()
    result = det.qrtest("qrcode.avi")
    print(str(result))
    
