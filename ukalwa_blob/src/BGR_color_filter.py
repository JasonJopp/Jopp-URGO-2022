import numpy as np
import cv2 as cv

# Gets new position of trackbar when bar is moved
def val(x):
	#print(x)
	pass
	
def create_BGR_filter():
	# Dictates the size of the window created, based on array
	img = np.ones((1,400,1), np.uint8)
	
	cv.namedWindow('Color Settings')

	# Creates the different trackbars
	# <bar name>, <win location>, <init val>, <max val>, <rtrn bar val>
	cv.createTrackbar('R Low', 'Color Settings', 0, 255, val)
	cv.createTrackbar('R High', 'Color Settings', 255, 255, val)
	cv.createTrackbar('B Low', 'Color Settings', 0, 255, val)
	cv.createTrackbar('B High', 'Color Settings', 255, 255, val)
	cv.createTrackbar('G Low', 'Color Settings', 0, 255, val)
	cv.createTrackbar('G High', 'Color Settings', 255, 255, val)

	# Tells user how to close program
	print("Press 'q' to quit.")
	
	# Displays window until 'q' is pressed
	while(True):
		cv.imshow("Color Settings",img)
		
		# Gets values of trackbars
		rLow = cv.getTrackbarPos('R Low', 'Color Settings')
		rHigh = cv.getTrackbarPos('R High', 'Color Settings')
		gLow = cv.getTrackbarPos('G Low', 'Color Settings')
		gHigh = cv.getTrackbarPos('G High', 'Color Settings')
		bLow = cv.getTrackbarPos('B Low', 'Color Settings')
		bHigh = cv.getTrackbarPos('B High', 'Color Settings')
		
		# Applies trackbar values to arrays for mask
		bgr_low = np.array([bLow,gLow,rLow])
		bgr_high = np.array([bHigh,gHigh,rHigh])
		
		if cv.waitKey(1) & 0xFF == ord('q'):
			break
	
	# Closes all windows
	cv.destroyAllWindows
