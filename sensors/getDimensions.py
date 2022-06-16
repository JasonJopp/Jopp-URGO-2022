import cv2 as cv

def getResolution():
    """Prints the dimensions of a cameras frame (resolution)."""
    capture = cv.VideoCapture(0)
    if not capture.isOpened():
        print("Could not find camera.")
        exit()

    ret, frame = capture.read()
    h, w, c, = frame.shape

    print('Width:', w)
    print('Height:', h)
    print('Channel:', c)

getResolution()