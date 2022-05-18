import numpy as np
import cv2 as cv

# Gets new position of trackbar when bar is moved
def val(x):
	print(x)
	
def create_BGR_filter():
	# Dictates the size of the window created, based on array
	img = np.ones((1,400,1), np.uint8)
	
	cv.namedWindow('Color Settings')

	# Creates the different trackbars
	# <bar name>, <win location>, <init val>, <max val>, <rtrn bar val>
	cv.createTrackbar('R', 'Color Settings', 0, 255, val)
	cv.createTrackbar('B', 'Color Settings', 0, 255, val)
	cv.createTrackbar('G', 'Color Settings', 0, 255, val)

	# Tells user how to close program
	print("Press 'q' to quit.")
	
	# Displays window until 'q' is pressed
	while(True):
		cv.imshow("Color Settings",img)
		if cv.waitKey(1) & 0xFF == ord('q'):
			break
	
	
	# Closes all windows
	cv.destroyAllWindows

create_BGR_filter()
