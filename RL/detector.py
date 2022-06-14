import numpy as np
import cv2 as cv

# Sets global variables
hLow = 150
hHigh = 190
sLow = 80
sHigh = 255
vLow = 0
vHigh = 255
# Used to calculate the rolling average of coordinates
coords = []
avgXY = (0,0)

def createDetector():
    """
    Creates a detector, which holds the parameters for blob detection.
    Detector should only need to be made once after params are set.
    """
    # Brings in global variables
    global vLow
    global vHigh
    
    # Initializes thresholds for blob detection
    params = cv.SimpleBlobDetector_Params()

    # Sets blob detection parameters to trackbar
    params.minThreshold = vLow
    params.maxThreshold = vHigh
    params.minArea = 100
    params.maxArea = 1000000
    params.minCircularity = 0.3

    # This prevents errors with threshold step size
    # Step size must be less than than diff of vHigh and vLow
    # Step size also cannot be zero
    stepSize = abs(vHigh-vLow)
    threshStep = 254
    if (threshStep >= stepSize):
        if stepSize < 2:
            params.thresholdStep = 1
        else:
            params.thresholdStep = stepSize - 1
    elif (threshStep > 0):
        params.thresholdStep = 254

    # Creates the detector object based on the set params
    detector = cv.SimpleBlobDetector_create(params)

    return detector

def blobDetector(frame, detector):
    """
    Takes a BGR frame, converts it to HSV, creates a mask,
    then returns the coordinates of the blob found.
    """
    
    # Brings in global variables
    global hLow
    global hHigh
    global sLow
    global sHigh
    global vLow
    global vHigh
    global coords
    global avgXY

    # Converts BGR frame to HSV frame, because HSV is less sensitive to light changes
    hsvFrame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    # Generates mask using hsvFrame, and HSV parameters
    mask = cv.inRange(hsvFrame, (hLow, sLow, vLow), (hHigh, sHigh, vHigh))

    # Sets "opening"/"closing" of mask (Morphology)
    kernel = np.ones((5,5), np.uint8)
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel, iterations = 2) # Removes false positives
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel) # Removes false negatives

    # Detects blobs, creates frame for displaying blobs
    blobs = detector.detect(cv.bitwise_not(mask))
    
    # Creates rolling average of coordinates, dependant on windows_size
    # NOTE: Need to moving rolling average calculation outside of this module
    if 0 < len(blobs) < 2:	
        xCoord = round(blobs[-1].pt[0])
        yCoord = round(blobs[-1].pt[1])
         # Change window size here to make examined array larger
        while (len(coords) >= 5):
            del coords[0]
        coords.append((xCoord,yCoord))
        
    else:
        while (len(coords) >= 5):
            del coords[0]
        coords.append((np.nan,np.nan))
    # Averages and rounds coordinates along 0 axis, ignoring NaNs
    avgXY = np.round_(np.nanmean(coords, axis=0))

    return avgXY


