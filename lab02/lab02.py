import cv2
import numpy as np

def bgr_histogram(img):
    height = img.shape[0]
    width = img.shape[1]
    new_image = np.zeros((height, width, img.shape[2])).astype(np.uint8)

    """for h in range(height):
        for w in range(width):
            new_image[h,w,2] = img[h,w,2]"""

    
    for c in range(3):
        histo = np.zeros(256,dtype=np.float16)
        for h in range(height):
            for w in range(width):
                h_value = img[h,w,c]
                histo[h_value] += 1

        total_pixel = height * width 
        sum = 0
        for i in range(256):
            sum += histo[i]

        p = 1/sum
        norm_histogram = np.zeros(256,dtype=np.float16)

        # equalize
        norm_histogram[0] = histo[0]*p
        #print(norm_histogram[0])
        for i in range(1,256):
            norm_histogram[i] = norm_histogram[i-1]+histo[i]*p
            #print(c,norm_histogram[i])
        for i in range(256):
            norm_histogram[i] = round(256*norm_histogram[i])
        # remap 
        for h in range(height):
            for w in range(width):
                g = img[h,w,c]
                new_image[h,w,c]= norm_histogram[g]

    return new_image


def hsv_histogram(img):
    histo = np.zeros((256,),dtype=np.float16)
    height = img.shape[0]
    width = img.shape[1]
    new_image = np.zeros((height, width, img.shape[2])).astype(np.uint8)
    for h in range(height):
        for w in range(width):
            h_value = img[h,w]
            histo[h_value] += 1

    total_pixel = height * width
    p = 1/total_pixel
    norm_histogram = np.zeros((256,),dtype=np.float16)

    # equalize
    norm_histogram[0] = histo[0]*p
    for i in range(1,256):
        norm_histogram[i] = norm_histogram[i-1]+histo[i]*p
    for i in range(256):
        norm_histogram[i] = round(255*norm_histogram[i])

    # remap 
    for h in range(height):
        for w in range(width):
            g = img[h,w]
            new_image[h,w] = norm_histogram[g]

    return new_image


def main():
    img = cv2.imread("histogram.jpg")
    new_img = bgr_histogram(img)
    newer_img = hsv_histogram(img)
    cv2.imshow("Original",img)
    cv2.imshow("bgrhisto",new_img)
    cv2.imshow("histo",newer_img)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()