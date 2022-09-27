import cv2
import numpy as np
from math import floor,sqrt

def nn_interpolate(image, dimension):
    new_image = np.zeros((dimension[0], dimension[1], image.shape[2])).astype(np.uint8)

    enlarge_time = int(
        sqrt((dimension[0] * dimension[1]) / (image.shape[0]*image.shape[1])))
    print(enlarge_time)

    for i in range(dimension[0]):
        for j in range(dimension[1]):
            row = floor(i / enlarge_time)
            column = floor(j / enlarge_time)

            new_image[i, j] = image[row, column]

    print(img)
    print('=======')
    print(new_image)
    return new_image

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

    print(image)
    print("======")
    print(new_image)
    return new_image

img = cv2.imread('test.jpg') #(532,799,3)
size = img.shape
#nnImg = nn_interpolate(img,(size[0]*3,size[1]*3))
newImg = bilinear_interpolate(img,(size[0]*3,size[1]*3))
cv2.imshow("Original",img)
#cv2.imshow("nn",nnImg)
cv2.imshow("bil image",newImg)

cv2.waitKey(0)
cv2.destroyAllWindows()
