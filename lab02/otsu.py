import cv2
import numpy as np

def otsu_thresholding(image):
	bins_num = 256
	# Calculate the total number of pixel in each bin
	histogram, bin_edges = np.histogram(image, bins=bins_num)

	# Calculate centers of bins by bin edges
	bin_mids = (bin_edges[:-1] + bin_edges[1:]) / 2

	# Iterate over all thresholds (indices)
	cluster1 = np.cumsum(histogram)
	cluster2 = np.cumsum(histogram[::-1])[::-1]

	# Get the cluser1 means for all thresholds
	mean1 = np.cumsum(histogram * bin_mids) / cluster1

	# Get the cluster2 means for all thresholds
	mean2 = (np.cumsum((histogram * bin_mids)[::-1]) / cluster2[::-1])[::-1]

	# Get the inter class variance of all thresholds
	inter_class_variance = cluster1[:-1] * cluster2[1:] * (mean1[:-1] - mean2[1:]) ** 2

	# Maximize the inter_class_variance function val
	index_of_max_val = np.argmax(inter_class_variance)

	threshold = bin_mids[:-1][index_of_max_val]

	return threshold

def main():
	image = cv2.imread('otsu.jpg')
	threshold = otsu_thresholding(image)
	print(threshold)
	mask = image < threshold
	output = image.copy()
	output[mask] = 0
	output[~mask] = 255

	cv2.imwrite('output.jpg', output)
	cv2.imshow("My Image", output)
	cv2.waitKey(0)
	cv2.destroyAllWindows()





if __name__ == '__main__':
	main()
