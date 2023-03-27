import cv2
from baseDetector import CrossDetection
import numpy as np
import math


class DuctDetector():

    def __init__(self):
        pass

    def calibration(self, img, i):

        def nothing(x):
            pass

        cv2.namedWindow("Parâmetros")
        cv2.createTrackbar('h', 'Parâmetros',i[0][0],255,nothing)
        cv2.createTrackbar('s', 'Parâmetros',i[0][1],255,nothing)
        cv2.createTrackbar('v', 'Parâmetros',i[0][2],255,nothing)

        cv2.createTrackbar('H', 'Parâmetros',i[1][0],255,nothing)
        cv2.createTrackbar('S', 'Parâmetros',i[1][1],255,nothing)
        cv2.createTrackbar('V', 'Parâmetros',i[1][2],255,nothing)

        cv2.createTrackbar('Blur', 'Parâmetros', i[2][0], 100, nothing)
        cv2.createTrackbar('Erode', 'Parâmetros', i[2][1], 100, nothing)
        cv2.createTrackbar('Dilate', 'Parâmetros', i[2][2], 100, nothing)

        while True:

            h = cv2.getTrackbarPos('h', 'Parâmetros')
            s = cv2.getTrackbarPos('s', 'Parâmetros')
            v = cv2.getTrackbarPos('v', 'Parâmetros')

            H = cv2.getTrackbarPos('H', 'Parâmetros')
            S = cv2.getTrackbarPos('S', 'Parâmetros')
            V = cv2.getTrackbarPos('V', 'Parâmetros')

            blur_size = cv2.getTrackbarPos('Blur', 'Parâmetros')
            erode_size = cv2.getTrackbarPos('Erode', 'Parâmetros')
            dilate_size = cv2.getTrackbarPos('Dilate', 'Parâmetros')

            blur_value = (blur_size, blur_size)
            erode_kernel = np.ones((erode_size, erode_size), np.float32)
            dilate_kernel = np.ones((dilate_size, dilate_size), np.float32)

            lower = [h, s, v]
            upper = [H, S, V]

            lower_color = np.array(lower)
            upper_color = np.array(upper)

            if blur_size != 0:
                blur = cv2.blur(img,blur_value)
            else:
                blur = img

            hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, lower_color, upper_color)

            imgMask = cv2.bitwise_and(blur, blur, mask=mask)
            
            dilate = cv2.dilate(imgMask, dilate_kernel)
            erode = cv2.erode(dilate, erode_kernel)

            cv2.imshow('color calibration', erode)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

        cv2.destroyAllWindows()
        parameters = [[h, s, v], [H, S, V], [blur_size, erode_size, dilate_size]]
        print(parameters)
        return parameters


    def apply_filters(self, img, parameters):
        p = parameters

        lower = p[0]
        upper = p[1]

        blur = (p[2][0], p[2][0])
        erode = p[2][1]
        dilate = p[2][2]

        lower_color = np.array(lower)
        upper_color = np.array(upper)

        # Blur
        if p[2][0] != 0:
            img = cv2.blur(img,blur)

        # HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Color filter
        mask = cv2.inRange(hsv, lower_color, upper_color)
        img_mask = cv2.bitwise_and(img, img, mask=mask)

        # Erode and dilate
        erode_kernel = np.ones((erode, erode), np.float32)
        dilate_kernel = np.ones((dilate, dilate), np.float32)
        dilate = cv2.dilate(img_mask, dilate_kernel)
        erode = cv2.erode(dilate, erode_kernel)

        # Canny filter
        canny = cv2.Canny(erode, 200, 300)

        return canny


    def duct_analysis(self, img, p, z, fov):

        img_filter = self.apply_filters(img, p)

        lines = cv2.HoughLines(img_filter, 1, np.pi / 180, 150, None, 0, 0)

        thetas = []
        points = []
        if lines is not None:
            for i in range(0, len(lines)):
                rho = lines[i][0][0]
                theta = lines[i][0][1]
                a = math.cos(theta)
                b = math.sin(theta)
                x0 = a * rho
                y0 = b * rho
                pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
                pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
                cv2.line(img, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)
                thetas.append(theta)
                points.append((x0, y0))
                cv2.circle(img, (int(x0), int(y0)), 4, (0, 0, 255), -1)

        if len(thetas) == 2:
            ang_rad = (thetas[0] + thetas[1])/2
            ang_deg = ang_rad * 180 / np.pi
            dist = math.dist(points[0], points[1])
            print(f"O tubo está com inclinação de {round(ang_deg, 1)}°.")
            print(f"Arena de 8x8 metros, logo: comprimento do tubo = {round(8/math.cos(ang_rad), 2)} metros.")
            print(f"Largura do tubo = {int(dist)} pixels.")

        cv2.imshow('filter', img)
        cv2.waitKey(0)


if __name__ == '__main__':

    drone_height = 2
    camera_fov = 1

    image = cv2.imread('tubo2.png')
    cv2.imshow('frame', image)
    cv2.waitKey(0)

    ductAnalysis = DuctDetector()
    
    initial = [[0, 255, 255], [24, 255, 255], [0, 0, 0]]

    parameters = ductAnalysis.calibration(image, initial)

    ductAnalysis.duct_analysis(image, parameters, drone_height, camera_fov)
