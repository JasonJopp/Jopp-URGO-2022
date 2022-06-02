import cv2

capture = cv2.VideoCapture(0)

# Checks if camera is found
if not capture.isOpened():
	print("Cannot find camera")
	exit()

print(capture.grab())

while(True):

    _, frame = capture.retrieve()

    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

capture.release()
cv2.destroyAllWindows()
