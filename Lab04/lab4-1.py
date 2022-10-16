import cv2
import numpy as np

nCapImages = 50
nROWS, nCOLS = 9, 6
PD_OBJECT_POINTS = np.zeros((nCOLS*nROWS, 3), np.float32)
PD_OBJECT_POINTS[:,:2] = np.mgrid[0:nROWS, 0:nCOLS].T.reshape(-1,2)

winSize = (11, 11)
zeroZone = (-1, -1)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)

def main():
    cap = cv2.VideoCapture(0) # device
    obj_points = [] # 3d point in real world space
    img_points = [] # 2d points in image plane.
    nSaveImg = 0

    while True:
        # If the captured image is enough, then break the loop
        if len(img_points) >= nCapImages:
            break

        # Read the frame from the device    
        ret, frame = cap.read()

        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Find the corners from chess board
        ret, corners = cv2.findChessboardCorners(gray, (nROWS, nCOLS))

        # If the corners are found, add object and image points
        if ret:
            obj_points.append(PD_OBJECT_POINTS)
            new_corners = cv2.cornerSubPix(gray, corners, winSize, zeroZone, criteria)
            img_points.append(new_corners)

            # Draw and display the corners # DBG
            img = cv2.drawChessboardCorners(frame, (nROWS, nCOLS), new_corners, ret)
            cv2.imshow('img', img)
            cv2.waitKey(33)

        cv2.imshow('frame', frame) # DBG: show image
        cv2.waitKey(33) # DBG: show image

    ret, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None) # DBG: remark

    filename = "./calibrate.xml"
    f = cv2.FileStorage(filename, cv2.FILE_STORAGE_WRITE)
    f.write("intrinsic", cameraMatrix)
    f.write("distortion", distCoeffs)
    f.release()


    cv2.destroyAllWindows()
if __name__ == '__main__':
    main()