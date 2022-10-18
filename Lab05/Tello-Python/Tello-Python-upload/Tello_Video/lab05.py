import cv2
import numpy as np
#import tello
import time
import math
from djitellopy import Tello
from pyimagesearch.pid import PID

font                   = cv2.FONT_HERSHEY_SIMPLEX
bottomLeftCornerOfText = (10,500)
fontScale              = 1
fontColor              = (255,255,255)
thickness              = 1
lineType               = 2



# 59F41C
def main():
    # Tello
    drone = Tello()
    drone.connect()
    #time.sleep(10)
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
    parameters = cv2.aruco.DetectorParameters_create()
    
    i=0
    while True:
        drone.streamon()
        frame = drone.get_frame_read()
        frame = frame.frame
        
        markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)
        if(markerIds != None):
            
            frame = cv2.aruco.drawDetectedMarkers(frame, 
                markerCorners, markerIds)
            
            #Pose estimation for single markers. 
            fs = cv2.FileStorage("calibrate_drone_50.xml", cv2.FILE_STORAGE_READ)
            intrinsic = fs.getNode("intrinsic").mat()
            distortion = fs.getNode("distortion").mat()
            rvec, tvec, _objPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorners,15, intrinsic, distortion) 
            print(tvec)
            frame = cv2.aruco.drawAxis(frame, intrinsic, distortion, rvec, tvec, 0.1)
        
            coord_str = f"x: {tvec[0][0][0]}, y: {tvec[0][0][1]}, z: {tvec[0][0][2]}"

            cv2.putText(frame, coord_str, 
                bottomLeftCornerOfText, 
                font, 
                fontScale,
                fontColor,
                thickness,
                lineType)
        
        cv2.imshow("drone", frame)

        key = cv2.waitKey(1)
        i+=1
        if(i>500):
            break
    
    #cv2.destroyAllWindows()



if __name__ == '__main__':
    main()