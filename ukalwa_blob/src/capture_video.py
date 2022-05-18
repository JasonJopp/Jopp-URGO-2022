import cv2 as cv

def create_video_feed():

	# Selects camera to use
	capture = cv.VideoCapture(0)

	# Checks if camera is found
	if not capture.isOpened():
		print("Cannot find camera")
		exit()

	while True:
		# Captures each frame
		ret, frame = capture.read()
		
		# Displays frames until 'q' is pressed
		cv.imshow('frame',frame)
		if cv.waitKey(1) & 0xFF == ord('q'):
			break
		
	# Releases capture
	capture.release()
	cv.destroyAllWindows

create_video_feed()
