from baseDetector import CrossDetection
import cv2


def centralize_on_cross(drone):

    '''
    This function, when started, tries to centralize the drone on a cross base marker.
    If no markers are detected, the drone is shaked to try a different angle of view.
    Without detection for a long period, the function is ended.

    Function parameters:
    drone -> MAV2 object

    '''

    detection = CrossDetection()
    vs = cv2.VideoCapture(0)

    TARGET = (int(vs.read().shape[1]/2), int(drone.read().shape[0]/2))

    is_centralized = False
    while not is_centralized:
        
        # Loop over frames to search for markers
        # If no markers were found, tries to shake the drone
        cross_detected = False
        timer = 0
        no_detection = 0
        
        while not cross_detected:
            
            parameters = [[0, 0, 0], [255, 255, 255], [1, 0, 0]]

            _,frame = vs.read()
            list_of_bases = detection.base_detection(frame, parameters)

            if len(list_of_bases) > 0:
                cross_detected = True
                timer = 0

            if timer > 100:
                print("No visible bases, shaking drone...")
                drone.shake()
                timer = 0
                no_detection += 1

            if no_detection > 5:
                print("Aruco not found...")
                return

            timer += 1

        base = list_of_bases[0]

        # Calculate the PID errors
        delta_y = TARGET[0] - base[0]
        delta_x = TARGET[1] - base[1]

        # Adjust velocity
        drone.camera_pid(delta_x, delta_y, 0)

        # End centralization if the marker is close enough to the camera center
        if ((delta_x)**2 + (delta_y)**2)**0.5 < 100:
            # drone.set_vel(0, 0, 0)
            is_centralized = True
            print(f"Centralized! x: {delta_x} y: {delta_y}")
                
            return
        
        
if __name__ == '__main__':

    import sys
    import os
    from MAV_ardupilot import MAV2
    import rospy

    rospy.init_node("centralization")
    mav = MAV2()
    mav.takeoff(1.5)
    rospy.sleep(5)
    centralize_on_cross(mav)
