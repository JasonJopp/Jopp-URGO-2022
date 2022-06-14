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
    print(avgXY)
