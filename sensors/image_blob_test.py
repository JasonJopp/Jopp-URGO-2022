import numpy as np
import cv2 as cv

# Return function for trackbars, sets changes flag to True
changes = True

def change_detector(x):
	global changes
	changes = True

# Function for responding to mouse click events
def click_event(event, x, y, flags, param):
	# Runs on left click
	if event == cv.EVENT_LBUTTONDOWN:
		print(x, " ", y)
				

def combined():
	global changes
	
	# Initializes thresholds for blob detection
	params = cv.SimpleBlobDetector_Params()
		
	# Dictates the size of the window created, based on array
	img = np.ones((1,509,1), np.uint8)
	
	cv.namedWindow('Settings')

	# Creates the different trackbars for blob detection thresholds
	# <bar name>, <win location>, <init val>, <max val>, <rtrn bar val>
	cv.createTrackbar('Low', 'Settings', 0, 255, change_detector)
	cv.createTrackbar('High', 'Settings', 255, 255, change_detector)
	cv.createTrackbar('Min Area', 'Settings', 0, 200000, change_detector)
	cv.createTrackbar('Max Area', 'Settings', 200000, 200000, change_detector)
	cv.createTrackbar('Filter Circularity?', 'Settings', 0, 1, change_detector)
	cv.createTrackbar('Min Circ', 'Settings', 0, 1000, change_detector)
	cv.createTrackbar('Max Circ', 'Settings', 1000, 1000, change_detector)
	

	# Tells user how to close program
	print("Press 'q' to quit.")
	
	# Flag for initial setup of while loop for windows
	# This flag is for things I only want to run once in the loop
	init_window = True
	
	# Displays window until 'q' is pressed
	while(True):
		# Captures each image as if it were a video frame
		frame = cv.imread("cvShapes.png", cv.IMREAD_COLOR)
		
		# Creates GRAY frame
		grayFrame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
		
		# Only creates a new detector if changes were made to the params
		# Saves around 4-10% CPU usage, must test with tracking in video
		if changes:
			changes = False
			# Gets trackbar locations for mask
			grayLow = cv.getTrackbarPos('Low', 'Settings')
			grayHigh = cv.getTrackbarPos('High', 'Settings')
		
			# Sets blob detection parameters to trackbar
			params.minThreshold = grayLow
			params.maxThreshold = grayHigh
			params.minArea = cv.getTrackbarPos('Min Area', 'Settings')
			params.maxArea = cv.getTrackbarPos('Max Area', 'Settings')
			
			# Uses circularity circ switch is set to on when running
			if (cv.getTrackbarPos('Filter Circularity?', 'Settings') == 1):
				params.filterByCircularity = True
				params.minCircularity = cv.getTrackbarPos('Min Circ', 'Settings')/1000
				params.maxCircularity = cv.getTrackbarPos('Max Circ', 'Settings')/1000
			else:
				params.filterByCircularity = False
			
			# Creates detector using set params
			detector = cv.SimpleBlobDetector_create(params)
		
		# Creates mask for grayFrame
		mask = cv.inRange(grayFrame, grayLow, grayHigh)
		
		# Uses mask to create new frame with detected blobs
		blobs = detector.detect(cv.bitwise_not(mask))
		grayWithBlobs = cv.drawKeypoints(frame, blobs, np.array([]), (0,255,0), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
		
		# Resizes feeds to be smaller (orig: 640, 480)
		frame = cv.resize(frame, (252,189))
		mask = cv.resize(mask, (252,189))
		grayWithBlobs = cv.resize(grayWithBlobs, (500,375)) #.78125% original size
		
		# Displays different frames until 'q' is pressed
		cv.imshow('Settings',img)
		cv.imshow('frame',frame)
		cv.imshow('mask',mask)
		cv.imshow('grayWithBlobs', grayWithBlobs)
		
		# This section only runs things in this loop once, for setup
		if (init_window == True):
			# Moves windows prevent stacking
			cv.moveWindow("Settings", 0, 0)
			cv.moveWindow("frame", 512, 0)
			cv.moveWindow("mask", 768, 0)
			cv.moveWindow("grayWithBlobs", 512, 220)
			init_window = False
		
		# Function called when mouse event happens in frame
		cv.setMouseCallback('frame', click_event)
		
		# Waits for 'q' to close program
		if cv.waitKey(1) & 0xFF == ord('q'):
			break
	
	# Closes all windows
	cv.destroyAllWindows

# Runs function
combined()
