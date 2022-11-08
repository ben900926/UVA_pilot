import cv2
import numpy as np
import time
import math
from djitellopy import Tello
from pyimagesearch.pid import PID 

global isflying

def keyboard(self, key):
    #global is_flying
    print("key:", key)
    fb_speed = 40
    lf_speed = 40
    ud_speed = 50
    degree = 30
    if key == ord('1'):
        self.takeoff()
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

    # the level drone's at
    
    stage = -1
    while True:
        print(f"stage: {stage}")
        drone.streamon()
        frame = drone.get_frame_read()
        frame = frame.frame
        # if press button
        key = cv2.waitKey(1)
        cv2.imshow("drone", frame)
        if(key!=-1):
            keyboard(drone, key)
            if(key == ord('1') and stage == -1):
                stage = 0
        else:
            # Convert the color frame to grayscale for marker detection
            markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)

            # detect marker
            if(markerCorners is not None and markerIds is not None):
                frame = cv2.aruco.drawDetectedMarkers(frame, 
                    markerCorners, markerIds)
                #Pose estimation for single markers. 
                #rvec, tvec, _objPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorners, 19, intrinsic, distortion) 
                # [[1] [2]]
                
                if(markerIds[0,0]==1 or stage == 1):
                    print("marker 1!!!!!")
                    stage = 1
                    # z update  in the beginning z = 200
                    rvec, tvec, _objPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorners, 19, intrinsic, distortion)
                    z_update = tvec[0,0,2] - 150
                    max_speed = 40
                    # we need to do z
                    if(abs(z_update)>50):
                        print("org_z: " + str(z_update))
                        z_update = z_pid.update(z_update, sleep=0)
                        print("pid_z: "+str(z_update))
                        if(z_update > max_speed):
                            z_update = max_speed
                        elif(z_update < -1*max_speed):
                            z_update = -1*max_speed

                        #drone.move("forward", int(z_update))
                        print("z_update: ", z_update)
                        continue
                    
                    print(f"z: {tvec[0,0,2]}") # z at 305
                    if(tvec[0,0,2] < 200):
                        print("lifting up to id 1")
                        drone.move("up", 80)  # 125
                        time.sleep(1)
                        drone.move("forward", 120) # 250 too far, almost collide with id2
                        time.sleep(1)
                        drone.move("down", 80)
                        time.sleep(1)
                        # to id 2
                        drone.move("left", 40)
                        time.sleep(1)
                        drone.land()
                
            else:
                if(stage==0):
                    drone.move("up", 40)
                    time.sleep(1)
                #else:
                #drone.send_rc_control(0,0,0,0)
                

# 59F789
if __name__ == '__main__':
    main()