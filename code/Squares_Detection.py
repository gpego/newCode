# import the necessary packages
import numpy as np
import argparse
import cv2

cap = cv2.VideoCapture(r"C:\Users\giovi\Desktop\RoboCup\Sync2\newCode\vids\2023-02-20 ore 18.57.44_mod.mp4")

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", help = "path to the image")
args = vars(ap.parse_args())

while True:
	# load the image
	ret, image = cap.read()

	# define the list of boundaries
	boundaries = [
		([0, 100, 0], [255, 100, 255])
	]

	# loop over the boundaries
	for (lower, upper) in boundaries:
		# create NumPy arrays from the boundaries
		lower = np.array(lower, dtype = "uint8")
		upper = np.array(upper, dtype = "uint8")
		# find the colors within the specified boundaries and apply
		# the mask
		mask = cv2.inRange(image, lower, upper)
		output = cv2.bitwise_and(image, image, mask = mask)
		# show the images
		cv2.imshow("images", np.hstack([image, output]))
		cv2.waitKey(1)