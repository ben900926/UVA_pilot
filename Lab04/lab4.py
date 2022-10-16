import cv2
import numpy as np
  
# function to display the coordinates of
# of the points clicked on the image
"""def click_event(event, x, y, flags, params):
 
    # checking for left mouse clicks
    if event == cv2.EVENT_LBUTTONDOWN:
 
        # displaying the coordinates
        # on the Shell
        print(x, ' ', y)
 
        # displaying the coordinates
        # on the image window
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(img, str(x) + ',' +
                    str(y), (x,y), font,
                    1, (255, 0, 0), 2)
        cv2.imshow('image', img)
 
    # checking for right mouse clicks    
    if event==cv2.EVENT_RBUTTONDOWN:
 
        # displaying the coordinates
        # on the Shell
        print(x, ' ', y)
 
        # displaying the coordinates
        # on the image window
        font = cv2.FONT_HERSHEY_SIMPLEX
        b = img[y, x, 0]
        g = img[y, x, 1]
        r = img[y, x, 2]
        cv2.putText(img, str(b) + ',' +
                    str(g) + ',' + str(r),
                    (x,y), font, 1,
                    (255, 255, 0), 2)
        cv2.imshow('image', img)
"""

def bilinear_interpolate(image, dimension):
    height = image.shape[0]
    width = image.shape[1]

    scale_x = (width)/(dimension[1])
    scale_y = (height)/(dimension[0])

    new_image = np.zeros((dimension[0], dimension[1], image.shape[2])).astype(np.uint8)

    for k in range(3):
        for i in range(dimension[0]):
            for j in range(dimension[1]):
                x = (j+0.5) * (scale_x) - 0.5
                y = (i+0.5) * (scale_y) - 0.5

                x_int = int(x)
                y_int = int(y)

                # Prevent raising Index error
                x_int = min(x_int, width-2)
                y_int = min(y_int, height-2)

                x_diff = x - x_int
                y_diff = y - y_int

                a = image[y_int, x_int, k]
                b = image[y_int, x_int+1, k]
                c = image[y_int+1, x_int, k]
                d = image[y_int+1, x_int+1, k]
 

                # formula
                pixel = a*(1-x_diff)*(1-y_diff) + b*(x_diff) * \
                    (1-y_diff) + c*(1-x_diff) * (y_diff) + d*x_diff*y_diff

                new_image[i, j, k] = pixel.astype(np.uint8)

    return new_image

def wrap_myself(img, mat):
    new_image = np.zeros((img.shape[0], img.shape[1], img.shape[2])).astype(np.uint8)
    width = img.shape[1]
    height = img.shape[0]
    #for k in range(3):
    for y in range(img.shape[0]):
        for x in range(img.shape[1]):
            x_new = (mat[0][0]*x + mat[0][1]*y + mat[0][2]) / (mat[2][0]*x + mat[2][1]*y + mat[2][2])
            y_new = (mat[1][0]*x + mat[1][1]*y + mat[1][2]) / (mat[2][0]*x + mat[2][1]*y + mat[2][2])
            

            x_new = int(x_new)
            y_new = int(y_new)
            new_image[y_new,x_new,:] = img[y,x,:].astype(np.uint8)

    bili_img = bilinear_interpolate(new_image, (img.shape[0],img.shape[1]))

    return bili_img


# (277, 190), (265, 411), (618, 89), (626, 387)
def main():
 
    # reading the image
    #img = cv2.imread('screen.jpg', 1)
 
    cm_points = np.float32([[0,0], [640,0], [0,480], [640,480]])
    dsp_points2 = np.float32([[277-265, 190-89],  [618-265, 89-89], [265-265, 411-89], [626-265, 387-89]])
    dsp_points = np.float32([[277, 190],            [618, 89],      [265, 411],         [626, 387]]) #(x,y)?
    # projection mat
    mat = cv2.getPerspectiveTransform(cm_points, dsp_points2)
    
    """
    [[ 3.88091958e-01 -2.50000000e-02  1.20000000e+01]
    [-1.57812500e-01  4.27151998e-01  1.01000000e+02]
    [-4.09973207e-04 -1.03306424e-04  1.00000000e+00]]
    """
    
    # live cap
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    i= 0
    processed_scale = (370, 322)
    dsp_img = cv2.imread("screen.jpg", 1)
    while True:
        ret, img = cap.read()
        #processed = cv2.warpPerspective(img,mat, (370, 322)) # fixed!
        processed = wrap_myself(img, mat)
        # fit process
        for y in range(processed_scale[1]):
            for x in range(processed_scale[0]):
                if(89+y<190 and processed[y,x,0]==0 and processed[y,x,1]==0 and processed[y,x,2]==0):
                    continue
                if(89+y>387 and processed[y,x,0]==0 and processed[y,x,1]==0 and processed[y,x,2]==0):
                    continue
                dsp_img[89+y, 265+x, :] = processed[y,x,:]
        
        i+=1
        cv2.imshow('frame', dsp_img)
        if cv2.waitKey(100) & 0xFF == ord('q'):
            break

    # wait for a key to be pressed to exit
    # close the window
    cv2.destroyAllWindows()


if __name__=="__main__":
    main()
