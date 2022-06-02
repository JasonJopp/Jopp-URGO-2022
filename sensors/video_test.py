import cv2 as cv
import numpy as np

# Selects camera to use
capture = cv.VideoCapture(index = 0)
	
# Checks if camera is found
if not capture.isOpened():
	print("Cannot find camera")
	exit()

capture.set(cv.CAP_PROP_FRAME_WIDTH, 20)
capture.set(cv.CAP_PROP_FRAME_HEIGHT, 10)

while(True):
	# Captures each frame, converts from BGR to HSV colorcode
	ret, frame = capture.read()

	if (ret == False) and (frame == None):
		print("Frame was not read in correctly.")
	
	else:
		cv.imshow('frame',frame)
	
	# Waits for 'q' to close program
	if cv.waitKey(1) & 0xFF == ord('q'):
		break

# Releases capture, closes all windows
capture.release()
cv.destroyAllWindow
