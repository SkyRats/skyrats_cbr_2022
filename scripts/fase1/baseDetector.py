from imutils.perspective import four_point_transform
from imutils.video import VideoStream
import numpy as np
import cv2
import time

'''
This script was made for cross shaped CBR base detection with OpenCV.

1 - filters are applied to the original frame;
2 - square shapes are indentified as potential bases;
3 - the potential figures are croped using four_point_transform;
4 - otsu thresholding is applied to the cropped image;
5 - shapes are identified inside the potential squares;
6 - if 3 or more shapes are identified with the same center -> base;
7 - a list of (x, y) position of all the bases on the frame is returned.

'''

class CrossDetection:
    def __init__(self):
        pass

    # State of art SkyRats color calibration function
    def calibration(self, cam, i):

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

            img = cam.read()

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


    # Apply filters with given parameters on image
    def aply_filters(self, img, parameters):
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


    # Search for square shapes on image
    def find_potentials(self, image):
        
        contours, ret = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        shapes = []
        for contour in contours:
            approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)
            
            # 4 sides and not so small (reduce noise)
            if len(approx) == 4 and cv2.arcLength(contour,True) > 200:
                shapes.append(approx)
        
        return shapes


    # Verify if given points are near to each other
    def cluster(self, shapes):

        cluster = []
        LIM = 10
        i = 0
        while i < len(shapes):
            coord1 = shapes[i]
            cluster = [coord1]
            x_tot = coord1[0]
            y_tot = coord1[1]
            x_min = coord1[0] - LIM
            x_max = coord1[0] + LIM
            y_min = coord1[1] - LIM
            y_max = coord1[1] + LIM
            for j in range(len(shapes) - (i+1)):
                coord2 = shapes[j + (i+1)]
                if x_min <= coord2[0] <= x_max and y_min <= coord2[1] <= y_max:
                    cluster.append(coord2)
                    x_tot += coord2[0]
                    y_tot += coord2[1]
            if len(cluster) >= 3:
                return True
            i += 1
        return False


    # Function to verify if a given potential image really has a base inside
    def verify(self, shape, image):

        square = four_point_transform(image, shape.reshape(4, 2))
        contours, ret = cv2.findContours(
            square, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        shapes = []
        for contour in contours:

            approx = cv2.approxPolyDP(
                contour, 0.01 * cv2.arcLength(contour, True), True)
            
            if len(approx) < 20 and cv2.arcLength(contour,True) > 100:
                M = cv2.moments(contour)
                if M['m00'] != 0.0:
                    x = int(M['m10']/M['m00'])
                    y = int(M['m01']/M['m00'])
                    shapes.append((x, y))

        return self.cluster(shapes)


    # Final function: image + parameters -> list of base centers (x, y)
    def base_detection(self, img, parameters):

        img_filter = self.aply_filters(img, parameters)

        list_of_potentials = self.find_potentials(img_filter)

        result = []
        tol = 5
        for potential in list_of_potentials:

            if self.verify(potential, img_filter):
                M = cv2.moments(potential)
                if M['m00'] != 0.0:
                    x = int(M['m10']/M['m00'])
                    y = int(M['m01']/M['m00'])
                    new_cross = True
                    for point in result:
                        if (point[0]-tol) <= x <= (point[0]+tol) and (point[1]-tol) <= y <= (point[1]+tol):
                            new_cross = False
                    if new_cross:
                        result.append((x,y))
        return result


if __name__ == '__main__':
    import psutil

    # If you don't want video feedback -> False:
    DEBUG = True
    
    # For cpu benchmark analysis:
    BENCHMARK = True
    
    detection = CrossDetection()

    # Video webcam test
    print("[INFO] starting video stream...")
    vs = VideoStream(src=0).start()

    time.sleep(2.0)

    # Initial calibration parameters
    initial = [[94, 70, 0], [255, 255, 255], [0, 0, 0]]

    if DEBUG:
        parameters = detection.calibration(vs, initial)
    else:
        parameters = initial
    
    i = 0
    cpu_usage = 0
    
    while True:
        frame = vs.read()
        list_of_bases = detection.base_detection(frame, parameters)
        
        if BENCHMARK:
            cpu_usage += psutil.cpu_percent()
            i += 1
            if i > 1000:
                break

        if DEBUG:
            for pixel in list_of_bases:
                cv2.circle(frame, pixel, 5, (0, 0, 255), -1)

            cv2.imshow('shapes', frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
        else:
            print(list_of_bases)
    
    print(f"CPU usage: {round(cpu_usage/i, 1)}%")
