# TESTE 1 - FASE 1

from baseDetector import CrossDetection
import cv2
import rospy

import cv2
from numpy import arange, sqrt, arctan, sin, tan, zeros, array, meshgrid, pi
from numpy import argwhere, hypot

import math


class Defisheye:
    """
    Defisheye
    fov: fisheye field of view (aperture) in degrees
    pfov: perspective field of view (aperture) in degrees
    xcenter: x center of fisheye area
    ycenter: y center of fisheye area
    radius: radius of fisheye area
    angle: image rotation in degrees clockwise
    dtype: linear, equalarea, orthographic, stereographic
    format: circular, fullframe
    """

    def __init__(self, infile, **kwargs):
        vkwargs = {"fov": 180,
                   "pfov": 120,
                   "xcenter": None,
                   "ycenter": None,
                   "radius": None,
                   "angle": 0,
                   "dtype": "equalarea",
                   "format": "fullframe"
                   }
        self._start_att(vkwargs, kwargs)

        # if type(infile) == str:
        #     _image = cv2.imread(infile)
        # # elif type(infile) == ndarray:
        # #     _image = infile
        # else:
        #     print("oi")
        #     return

        _image = infile
        width = _image.shape[1]
        height = _image.shape[0]
        xcenter = width // 2
        ycenter = height  // 2

        dim = min(width, height)
        x0 = xcenter - dim // 2
        xf = xcenter + dim // 2
        y0 = ycenter - dim // 2
        yf = ycenter + dim // 2

        self._image = _image[y0:yf, x0:xf, :]

        self._width = self._image.shape[1]
        self._height = self._image.shape[0]

        if self._xcenter is None:
            self._xcenter = (self._width - 1) // 2

        if self._ycenter is None:
            self._ycenter = (self._height - 1) // 2

    def _map(self, i, j, ofocinv, dim):

        xd = i - self._xcenter
        yd = j - self._ycenter

        rd = hypot(xd, yd)
        phiang = arctan(ofocinv * rd)

        if self._dtype == "linear":
            ifoc = dim * 180 / (self._fov * pi)
            rr = ifoc * phiang
            # rr = "rr={}*phiang;".format(ifoc)

        elif self._dtype == "equalarea":
            ifoc = dim / (2.0 * sin(self._fov * pi / 720))
            rr = ifoc * sin(phiang / 2)
            # rr = "rr={}*sin(phiang/2);".format(ifoc)

        elif self._dtype == "orthographic":
            ifoc = dim / (2.0 * sin(self._fov * pi / 360))
            rr = ifoc * sin(phiang)
            # rr="rr={}*sin(phiang);".format(ifoc)

        elif self._dtype == "stereographic":
            ifoc = dim / (2.0 * tan(self._fov * pi / 720))
            rr = ifoc * tan(phiang / 2)

        rdmask = rd != 0
        xs = xd.copy()
        ys = yd.copy()

        xs[rdmask] = (rr[rdmask] / rd[rdmask]) * xd[rdmask] + self._xcenter
        ys[rdmask] = (rr[rdmask] / rd[rdmask]) * yd[rdmask] + self._ycenter

        xs[~rdmask] = 0
        ys[~rdmask] = 0

        xs = xs.astype(int)
        ys = ys.astype(int)
        return xs, ys

    def convert(self):
        if self._format == "circular":
            dim = min(self._width, self._height)
        elif self._format == "fullframe":
            dim = sqrt(self._width ** 2.0 + self._height ** 2.0)

        if self._radius is not None:
            dim = 2 * self._radius

        # compute output (perspective) focal length and its inverse from ofov
        # phi=fov/2; r=N/2
        # r/f=tan(phi);
        # f=r/tan(phi);
        # f= (N/2)/tan((fov/2)*(pi/180)) = N/(2*tan(fov*pi/360))

        ofoc = dim / (2 * tan(self._pfov * pi / 360))
        ofocinv = 1.0 / ofoc

        i = arange(self._width)
        j = arange(self._height)
        i, j = meshgrid(i, j)

        xs, ys, = self._map(i, j, ofocinv, dim)
        img = self._image.copy()

        img[i, j, :] = self._image[xs, ys, :]
        # cv2.imwrite(outfile, img)
        return img

    def _start_att(self, vkwargs, kwargs):
        """
        Starting atributes
        """
        pin = []

        for key, value in kwargs.items():
            if key not in vkwargs:
                raise NameError("Invalid key {}".format(key))
            else:
                pin.append(key)
                setattr(self, "_{}".format(key), value)

        pin = set(pin)
        rkeys = set(vkwargs.keys()) - pin
        for key in rkeys:
            setattr(self, "_{}".format(key), vkwargs[key])


###############################################################

def first(list):
    first = list[0]
    min_sum = first[0]**2 + first[1]**2
    for i in list[1:]:
        sum = i[0]**2 + i[1]**2
        if sum < min_sum:
            min_sum = sum
            first = i
    return first


def centralize_on_cross(drone):

    '''
    This function, when started, tries to centralize the drone on a cross base marker.
    If no markers are detected, the drone is shaked to try a different angle of view.
    Without detection for a long period, the function is ended.
    Function parameters:
    drone -> MAV2 object
    '''

    # Camera parameters
    dtype = 'linear'
    format = 'fullframe'
    fov = 140
    pfov = 90

    detection = CrossDetection()

    # Initialize camera
    vs = cv2.VideoCapture(0)
    _,frame = vs.read()
    
    # Fisheye correction
    defisheye = True
    if defisheye:
        frame = Defisheye(frame, dtype=dtype, format=format, fov=fov, pfov=pfov).convert()

    # Camera center
    TARGET = (int(frame.shape[1]/2), int(frame.shape[0]/2))
    
    cross_detected = False
    timer = 0
    no_detection = 0
    
    # Loop searching for bases to land
    while not cross_detected and not rospy.is_shutdown():
        
        # Filter parameters
        parameters = [[0, 0, 0], [255, 255, 255], [1, 0, 0]]

        # Read
        _,frame = vs.read()
        list_of_bases = detection.base_detection(frame, parameters)

        # If found a base
        if len(list_of_bases) > 0:
            print(f"Bases detectadas: {list_of_bases}")
            cross_detected = True
            timer = 0

        # After some detections, no bases were found
        if timer > 200:
            print("Nenhuma base visível...")
            timer = 0
            no_detection += 1

        # After a lot of tries, no bases were found
        if no_detection > 5:
            print("Nenhuma base encontrada...")
            return False

        timer += 1

    # Calculate errors
    relative_pos = []
    for base in list_of_bases:
        delta_y = TARGET[0] - base[0]
        delta_x = TARGET[1] - base[1]
        relative_pos.append((delta_x, delta_y))

        # Take the nearest base detected
        base = first(relative_pos)

    print(f"Base detectada, posição relativa à câmera: {base}")
    SETVEL = True
    # If already centralized
    if ((delta_x)**2 + (delta_y)**2)**0.5 < 80:
        drone.set_vel(0, 0, 0)
        print(f"Drone já centralizado! x: {delta_x} y: {delta_y}")

    # Sabendo a posição da base em relação à câmera, movimentar o drone na direção da base
    elif SETVEL:
        # Velocity vector
        V_mod = 0.15

        # Time for moving drone
        tempo = 1/100 * math.sqrt(base[0]**2 + base[1]**2)

        # Calculate de x and y components
        vel_x = base[0] / math.sqrt(base[0]**2 + base[1]**2) * V_mod
        vel_y = base[1] / math.sqrt(base[0]**2 + base[1]**2) * V_mod

        print(f"Movimentando drone -> x: {vel_x} / y: {vel_y} / tempo: {tempo}")

        # Set_vel time loop
        now = rospy.get_rostime()
        while not rospy.get_rostime() - now > rospy.Duration(secs=tempo) and not rospy.is_shutdown():
            drone.set_vel(vel_x, vel_y)
            drone.set_vel(0, 0, 0)
            
    return True
        
        
if __name__ == '__main__':

    import sys
    import os
    from MAV_ardupilot import MAV2
    import rospy

    rospy.init_node("centralization")
    mav = MAV2()

    # Takeoff
    mav.takeoff(2)
    rospy.sleep(5)

    print("centralize on cross")
    centralization = centralize_on_cross(mav)

    print("out of cross")
    if centralization(mav):
        print("Landing drone...")
        mav.land()
