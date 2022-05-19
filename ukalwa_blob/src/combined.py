import numpy as np
import cv2 as cv

# Gets new position of trackbar when bar is moved
def val(x):
	pass
	
def combined():
	# Selects camera to use
	capture = cv.VideoCapture(0)
	
	# Finds height/width of video capture (feed)
	# width = capture.get(cv.CAP_PROP_FRAME_WIDTH)
	# height = capture.get(cv.CAP_PROP_FRAME_HEIGHT)
	# print(height,width)
	
	# Checks if camera is found
	if not capture.isOpened():
		print("Cannot find camera")
		exit()
		
	# Dictates the size of the window created, based on array
	img = np.ones((1,400,1), np.uint8)
	
	cv.namedWindow('Color Settings')

	# Creates the different trackbars
	# <bar name>, <win location>, <init val>, <max val>, <rtrn bar val>
	cv.createTrackbar('H Low', 'Color Settings', 0, 255, val)
	cv.createTrackbar('H High', 'Color Settings', 255, 255, val)
	cv.createTrackbar('S Low', 'Color Settings', 0, 255, val)
	cv.createTrackbar('S High', 'Color Settings', 255, 255, val)
	cv.createTrackbar('V Low', 'Color Settings', 0, 255, val)
	cv.createTrackbar('V High', 'Color Settings', 255, 255, val)

	# Tells user how to close program
	print("Press 'q' to quit.")
	
	# Displays window until 'q' is pressed
	while(True):
		cv.imshow("Color Settings",img)
		
		# Captures each frame
		ret, frame = capture.read()
		
		# Creates HSV frame
		hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
		
		# Gets values of trackbars
		hLow = cv.getTrackbarPos('H Low', 'Color Settings')
		hHigh = cv.getTrackbarPos('H High', 'Color Settings')
		sLow = cv.getTrackbarPos('S Low', 'Color Settings')
		sHigh = cv.getTrackbarPos('S High', 'Color Settings')
		vLow = cv.getTrackbarPos('V Low', 'Color Settings')
		vHigh = cv.getTrackbarPos('V High', 'Color Settings')
		
		# Applies trackbar values to arrays for mask
		hsv_low = np.array([hLow,sLow,vLow])
		hsv_high = np.array([hHigh,sHigh,vHigh])
		
		# Creates the mask
		mask = cv.inRange(hsv, hsv_low, hsv_high)
		
		# Creates result frame w/ mask
		res = cv.bitwise_and(hsv, hsv, mask = mask)
		
		# Resizes feeds to be smaller (orig: 640, 480)
		frame = cv.resize(frame, (240,180))
		mask = cv.resize(mask, (240,180))
		
		# Displays different frames until 'q' is pressed
		cv.imshow('frame',frame)
		cv.imshow('mask', mask)
		cv.imshow('res', res)
		
		# Waits for 'q' to close program
		if cv.waitKey(1) & 0xFF == ord('q'):
			break
	
	# Releases capture
	capture.release()
	
	# Closes all windows
	cv.destroyAllWindows

combined()
