import cv2
import numpy as np
import time
import math
from pyimagesearch.pid import PID 


def main():

    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
    parameters = cv2.aruco.DetectorParameters_create()
    global isflying
    
    key = -1

    # parameters of camera calibration
    fs = cv2.FileStorage("calibrate_drone_50.xml", cv2.FILE_STORAGE_READ)
    intrinsic = fs.getNode("intrinsic").mat()
    distortion = fs.getNode("distortion").mat()
    
    z_pid = PID(kP=0.7, kI=0.0001, kD=0.1)
    y_pid = PID(kP=0.7, kI=0.0001, kD=0.1)
    yaw_pid = PID(kP=0.7, kI=0.0001, kD=0.1)

    yaw_pid.initialize()
    y_pid.initialize()
    z_pid.initialize()
    
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

    while True:

        ret, frame = cap.read()
        # if press button
        key = cv2.waitKey(100)
        if key & 0xFF == ord('q'):
            break

        #print(key)
        if(key!=-1):
            print(key)
        # Convert the color frame to grayscale for marker detection
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)
        # detect marker
        if(markerCorners is not None and markerIds is not None):
            print(markerIds)
            frame = cv2.aruco.drawDetectedMarkers(frame, 
                markerCorners, markerIds)
            
            #Pose estimation for single markers. 
            rvec, tvec, _objPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorners, 0.19, intrinsic, distortion) 
            #print(rvec, tvec)

            coord_str = f"y: {tvec[0][0][1]}, z: {tvec[0][0][2]}"
            #print(coord_str)

        
        
        
        cv2.imshow("drone", frame)
                

if __name__ == '__main__':

    main()
