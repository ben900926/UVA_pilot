from multiprocessing import connection
import cv2
import numpy as np
import time
import math
from djitellopy import Tello
from pyimagesearch.pid import PID 

def keyboard(self, key):
    #global is_flying
    print("key:", key)
    fb_speed = 40
    lf_speed = 40
    ud_speed = 50
    degree = 30
    if key == ord('1'):
        self.takeoff()
        #is_flying = True
    if key == ord('2'):
        self.land()
        #is_flying = False
    if key == ord('3'):
        self.send_rc_control(0, 0, 0, 0)
        print("stop!!!!")
    if key == ord('w'):
        self.send_rc_control(0, fb_speed, 0, 0)
        print("forward!!!!")
    if key == ord('s'):
        self.send_rc_control(0, (-1) * fb_speed, 0, 0)
        print("backward!!!!")
    if key == ord('a'):
        self.send_rc_control((-1) * lf_speed, 0, 0, 0)
        print("left!!!!")
    if key == ord('d'):
        self.send_rc_control(lf_speed, 0, 0, 0)
        print("right!!!!")
    if key == ord('z'):
        self.send_rc_control(0, 0, ud_speed, 0)
        print("down!!!!")
    if key == ord('x'):
        self.send_rc_control(0, 0, (-1) *ud_speed, 0)
        print("up!!!!")
    if key == ord('c'):
        self.send_rc_control(0, 0, 0, degree)
        print("rotate!!!!")
    if key == ord('v'):
        self.send_rc_control(0, 0, 0, (-1) *degree)
        print("counter rotate!!!!")
    if key == ord('5'):
        height = self.get_height()
        print(height)
    if key == ord('6'):
        battery = self.get_battery()
        print (battery)



def main():
    drone = Tello()
    drone.connect()
    #time.sleep(10)
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
    parameters = cv2.aruco.DetectorParameters_create()
    global isflying
    
    drone.takeoff()
    isflying = 1
    #is_takeoff = 0
    key = -1
    font                   = cv2.FONT_HERSHEY_SIMPLEX
    bottomLeftCornerOfText = (10,500)
    fontScale              = 1
    fontColor              = (255,255,255)
    thickness              = 1
    lineType               = 2

    while True:
        drone.streamon()
        frame = drone.get_frame_read()
        frame = frame.frame
        """
        if(not is_takeoff):
            drone.takeoff()
            is_takeoff = 1
        """
        # if press button
        print(key)
        if(key!=-1):
            print(key)
            keyboard(drone, key)
        
        markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)
        if(markerIds != None):
            frame = cv2.aruco.drawDetectedMarkers(frame, 
                markerCorners, markerIds)
            
            #Pose estimation for single markers. 
            fs = cv2.FileStorage("calibrate_drone_50.xml", cv2.FILE_STORAGE_READ)
            intrinsic = fs.getNode("intrinsic").mat()
            distortion = fs.getNode("distortion").mat()
            rvec, tvec, _objPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorners,15, intrinsic, distortion) 
            #print(rvec, tvec)
            
            z_pid = PID(kP=0.7, kI=0.0001, kD=0.1)
            y_pid = PID(kP=0.7, kI=0.0001, kD=0.1)
            yaw_pid = PID(kP=0.7, kI=0.0001, kD=0.1)

            yaw_pid.initialize()
            y_pid.initialize()
            z_pid.initialize()

            coord_str = f"y: {tvec[0][0][1]}, z: {tvec[0][0][2]}"
            cv2.putText(frame, coord_str, 
                bottomLeftCornerOfText, 
                font, 
                fontScale,
                fontColor,
                thickness,
                lineType)

            # z update
            z_update = tvec[0,0,2] - 100
            print("org_z: " + str(z_update))
            max_speed = 40
            # we need to do z
            if(abs(z_update)>20):
                z_update = z_pid.update(z_update, sleep=0)
                print("pid_z: "+str(z_update))
                if(z_update > max_speed):
                    z_update = max_speed
                elif(z_update < -1*max_speed):
                    z_update = -1*max_speed
                drone.send_rc_control(0, int(z_update), 0, 0)
            else:
                z_update = 0
            
            # y_update
            y_update = tvec[0,0,1] - 10
            print("org_y: " + str(y_update))
            if(abs(y_update)>10):
                y_update = y_pid.update(y_update, sleep=0)
                print("pid_y: "+str(y_update))
                if(y_update > max_speed):
                    y_update = max_speed
                elif(y_update < -1*max_speed):
                    y_update = -1*max_speed
            else:
                y_update = 0
                
            drone.send_rc_control(0, int(z_update), int(y_update), 0)
        # if not detect marker, stop
        else:
            drone.send_rc_control(0, 0, 0, 0)
            
        
        key = cv2.waitKey(1)
        cv2.imshow("drone", frame)
                
            


        
        

        




if __name__ == '__main__':
    main()