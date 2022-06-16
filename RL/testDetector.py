# NOTE: The purpose of this program is to test the OpenCV blob detection of an object.
# This program is not required for the machine learning research.

import numpy as np
import cv2 as cv
from detector import createDetector, blobDetector

capture = cv.VideoCapture(0)
if not capture.isOpened():
    print("Could not find camera.")
    exit()

detector = createDetector()

while True:
    ret, frame = capture.read()
    avgXY = blobDetector(frame, detector)
    if np.isnan(avgXY[0]):
        print("Wait, this isn't a number!")
    else:
        print(avgXY)
