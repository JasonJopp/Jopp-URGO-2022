import numpy as np
import cv2 as cv

# This flag tracks whether the detector needs to be reconstructed
# if new params are set for the detector
changes = True

# Return function for trackbars, sets changes flag to True
def change_detector(x):
	global changes
	changes = True

def combined():
	global changes

	# Array is used to calculate the rolling average of coordinates
	coords = []
	avgXY = (0,0)
	
	# Initializes thresholds for blob detection
	params = cv.SimpleBlobDetector_Params()
	
	# Selects camera to use
	capture = cv.VideoCapture(0, cv.CAP_ANY)
	
	# Checks if camera is found
	if not capture.isOpened():
		print("Cannot find camera")
		exit()
		
	# Dictates the size of the window created, based on array
	img = np.ones((1,509,1), np.uint8)
	
	cv.namedWindow('Settings')

	# Creates the different trackbars for blob detection thresholds
	# <bar name>, <win location>, <init val>, <max val>, <rtrn bar val>
	cv.createTrackbar('H Low', 'Settings', 0, 255, change_detector)
	cv.createTrackbar('H High', 'Settings', 255, 255, change_detector)
	cv.createTrackbar('S Low', 'Settings', 50, 255, change_detector)
	cv.createTrackbar('S High', 'Settings', 255, 255, change_detector)
	cv.createTrackbar('V Low', 'Settings', 0, 255, change_detector)
	cv.createTrackbar('V High', 'Settings', 255, 255, change_detector)
	cv.createTrackbar('Min Area', 'Settings', 300, 1000, change_detector)
	cv.createTrackbar('Max Area', 'Settings', 200000, 200000, change_detector)
	cv.createTrackbar('Min Circ', 'Settings', 150, 1000, change_detector)
	cv.createTrackbar('Max Circ', 'Settings', 1000, 1000, change_detector)
	cv.createTrackbar('Min Blob Dist', 'Settings', 0, 1000, change_detector)
	cv.createTrackbar('Thresh Step', 'Settings', 254, 254, change_detector)
	cv.createTrackbar('Window Size', 'Settings', 10, 50, change_detector)

	# Tells user how to close program
	print("Press 'q' to quit.")
	
	# Flag for initial setup of while loop for windows
	# This flag is for things I only want to run once in the loop
	init_window = True
	
	# Displays window until 'q' is pressed
	while(True):
		cv.imshow("Settings",img)
		
		# Captures each frame, converts from BGR to HSV colorcode
		ret, frame = capture.read()
		hsvFrame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
		
		# Only creates a new detector if changes were made to the params
		if changes:
			changes = False
			hLow = cv.getTrackbarPos('H Low', 'Settings')
			hHigh = cv.getTrackbarPos('H High', 'Settings')
			sLow = cv.getTrackbarPos('S Low', 'Settings')
			sHigh = cv.getTrackbarPos('S High', 'Settings')
			vLow = cv.getTrackbarPos('V Low', 'Settings')
			vHigh = cv.getTrackbarPos('V High', 'Settings')
			windowSize = cv.getTrackbarPos('Window Size', 'Settings')
			if windowSize == 0:
				windowSize = 1
		
			# Sets blob detection parameters to trackbar
			params.minThreshold = vLow
			params.maxThreshold = vHigh
			params.minArea = cv.getTrackbarPos('Min Area', 'Settings')
			params.maxArea = cv.getTrackbarPos('Max Area', 'Settings')
			params.minCircularity = cv.getTrackbarPos('Min Circ', 'Settings')/1000
			params.maxCircularity = cv.getTrackbarPos('Max Circ', 'Settings')/1000
			params.minDistBetweenBlobs = cv.getTrackbarPos('Min Blob Dist', 'Settings')
			# This prevents errors with threshold step size
			# Step size must be less than than diff of vHigh and vLow
			# Step size also cannot be zero
			stepSize = abs(vHigh-vLow)
			if (cv.getTrackbarPos('Thresh Step', 'Settings') >= stepSize):
				if stepSize < 2:
					params.thresholdStep = 1
				else:
					params.thresholdStep = stepSize - 1
			elif (cv.getTrackbarPos('Thresh Step', 'Settings') > 0):
				params.thresholdStep = cv.getTrackbarPos('Thresh Step', 'Settings')
			
			# Creates the detector object based on the set params
			detector = cv.SimpleBlobDetector_create(params)
		
		# Creates mask for hsvFrame, large CPU performance sink
		mask = cv.inRange(hsvFrame, (hLow, sLow, vLow), (hHigh, sHigh, vHigh))

		# Sets "opening"/"closing" of mask (Morphology)
		kernel = np.ones((5,5), np.uint8)
		mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel, iterations = 2) # Removes false positives
		mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel) # Removes false negatives
		
		# Detects blobs, creates frame for displaying blobs
		blobs = detector.detect(cv.bitwise_not(mask))
		hsvBlobs = cv.drawKeypoints(frame, blobs, np.array([]), (0,255,0), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
		
		# Waits for 'q' to close program
		if cv.waitKey(1) & 0xFF == ord('q'):
			break
		
		# Creates rolling average of coordinates, dependant on windows_size
		if 0 < len(blobs) < 2:	
			xCoord = round(blobs[-1].pt[0])
			yCoord = round(blobs[-1].pt[1])
			while (len(coords) >= windowSize): # Change window size here to make examined array larger
				del coords[0]
			coords.append((xCoord,yCoord))
			avgXY = np.round_(np.mean(coords, axis=0)) # Averages and rounds coordinates along 0 axis
		
		hsvBlobs = cv.circle(hsvBlobs, (int(avgXY[0]),int(avgXY[1])), 20, (255,0,0), 2)
		hsvBlobs = cv.circle(hsvBlobs, (320,210), 5, (0,0,255), 3)

		# Displays different frames until 'q' is pressed
		cv.imshow('mask',mask)
		cv.imshow('hsvBlobs', hsvBlobs)

		# This section only runs things in this loop once, for setup
		if (init_window == True):
			# Moves windows prevent stacking
			cv.moveWindow('Settings', 0, 40)
			cv.moveWindow('mask', 512, 40)
			cv.moveWindow('hsvBlobs', 1155, 40)
			init_window = False
		

			
	# Releases capture
	capture.release()
	
	# Closes all windows
	cv.destroyAllWindows

combined()
