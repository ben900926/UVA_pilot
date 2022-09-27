import numpy as np
import cv2

# B G R
def flag_keep_blue(image):
	width, height, channels = image.shape
	image_cpy = image.copy()

	for i in range(width):
		for j in range(height):
			B, G, R = int(image[i][j][0]), int(image[i][j][1]), int(image[i][j][2])
			if not (B > 70 and B * 0.8 > G and B * 0.8 > R):
				image_cpy[i][j][0] = (R+G+B) / 3
				image_cpy[i][j][1] = (R+G+B) / 3
				image_cpy[i][j][2] = (R+G+B) / 3

	return image_cpy


def flag_keep_blue_yellow(image):
	width, height, channels = image.shape
	image_cpy = image.copy()

	for i in range(width):
		for j in range(height):
			B, G, R = int(image[i][j][0]), int(image[i][j][1]), int(image[i][j][2])
			if not (B > 70 and B * 0.8 > G and B * 0.8 > R) and not (R > 75 and G > 75 and (R + G) * 0.19 > B):
				image_cpy[i][j][0] = (R+G+B) / 3
				image_cpy[i][j][1] = (R+G+B) / 3
				image_cpy[i][j][2] = (R+G+B) / 3

	return image_cpy

def main():
	image = cv2.imread('nctu_flag.jpg')
	#image1 = flag_keep_blue(image)
	image1 = flag_keep_blue_yellow(image)
	cv2.imshow("My Image", image1)
	cv2.waitKey(0)
	cv2.destroyAllWindows()
	cv2.imwrite('output-2.jpg', image1)


if __name__ == '__main__':
	main()
