import numpy as np
import cv2 as cv

# Gets new position of trackbar when bar is moved
def val(x):
	pass

# Function for responding to mouse click events
def click_event(event, x, y, flags, param):
	# Runs on left click
	if event == cv.EVENT_LBUTTONDOWN:
		print(x, " ", y)
				

def combined():
	# Initializes thresholds for blob detection
	params = cv.SimpleBlobDetector_Params()
	
	# Selects camera to use
	capture = cv.VideoCapture(0)
	
	# Checks if camera is found
	if not capture.isOpened():
		print("Cannot find camera")
		exit()
		
	# Dictates the size of the window created, based on array
	img = np.ones((1,509,1), np.uint8)
	
	cv.namedWindow('Settings')

	# Creates the different trackbars for blob detection thresholds
	# <bar name>, <win location>, <init val>, <max val>, <rtrn bar val>
	cv.createTrackbar('Low', 'Settings', 0, 255, val)
	cv.createTrackbar('High', 'Settings', 255, 255, val)
	cv.createTrackbar('Min Area', 'Settings', 0, 300000, val)
	cv.createTrackbar('Max Area', 'Settings', 300000, 300000, val)
	cv.createTrackbar('Filter Circularity?', 'Settings', 0, 1, val)
	cv.createTrackbar('Min Circ', 'Settings', 0, 1000, val)
	cv.createTrackbar('Max Circ', 'Settings', 1000, 1000, val)
	

	# Tells user how to close program
	print("Press 'q' to quit.")
	
	# Flag for initial setup of while loop for windows
	# This flag is for things I only want to run once in the loop
	init_window = True
	
	# Displays window until 'q' is pressed
	while(True):
		cv.imshow("Settings",img)
		
		# Captures each frame
		ret, frame = capture.read()
		
		# Creates GRAY frame
		grayFrame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
		
		grayLow = cv.getTrackbarPos('Low', 'Settings')
		grayHigh = cv.getTrackbarPos('High', 'Settings')
		
		# Creates mask for grayFrame
		mask = cv.inRange(grayFrame,grayLow, grayHigh)
		
		# Sets blob detection parameters to trackbar
		params.minThreshold = grayLow
		params.maxThreshold = grayHigh
		params.minArea = cv.getTrackbarPos('Min Area', 'Settings')
		params.maxArea = cv.getTrackbarPos('Max Area', 'Settings')
		if (cv.getTrackbarPos('Filter Circularity?', 'Settings') == 1):
			params.filterByCircularity = True
			params.minCircularity = cv.getTrackbarPos('Min Circ', 'Settings')/1000
			params.maxCircularity = cv.getTrackbarPos('Max Circ', 'Settings')/1000
		else:
			params.filterByCircularity = False
		
		#detector = cv.SimpleBlobDetector(params)
		detector = cv.SimpleBlobDetector_create(params)
		not_mask = cv.bitwise_not(mask)
		blobs = detector.detect(not_mask)
		grayWithBlobs = cv.drawKeypoints(frame, blobs, np.array([]), (0,0,255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
		
		# Resizes feeds to be smaller (orig: 640, 480)
		###frame = cv.resize(frame, (252,189))
		#mask = cv.resize(mask, (252,189))
		###result = cv.resize(result, (500,375)) #.78125% original size
		
		# Displays different frames until 'q' is pressed
		cv.imshow('frame',frame)
		cv.imshow('mask',mask)
		cv.imshow('grayWithBlobs', grayWithBlobs)
		
		# This section only runs things in this loop once, for setup
		if (init_window == True):
			# Moves windows prevent stacking
			cv.moveWindow("Settings", 0, 0)
			cv.moveWindow("frame", 512, 0)
			cv.moveWindow("result", 508, 0)
			init_window = False
		
		# Function called when mouse event happens in frame
		cv.setMouseCallback('frame', click_event)
		
		# Waits for 'q' to close program
		if cv.waitKey(1) & 0xFF == ord('q'):
			break
	
	# Releases capture
	capture.release()
	
	# Closes all windows
	cv.destroyAllWindows

combined()
