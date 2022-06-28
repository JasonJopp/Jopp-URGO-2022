# -----------------------------------------------------------------------------
# Code Purpose: Given a image or video frame, return the x,y coordinates and
# diameter of a detected object that matches the preset detector parameters.
# 
# createDetector() creates a detector object of the OpenCV (cv) library. 
# The detector doesn't detect blobs, it contains parameters for blob detection.
# Think of the detector as the lens to see specific blobs through. The detector
# only needs to be created once, unless you want to change params while the 
# program is running.
# 
# blobDetector() then takes the detector objects and a image/video frame, 
# examines the frame using the detector "lens" and returns the coordinates 
# (x,y) and diameter (pixels) of the detected blob. The rolling average is
# useful if there are some false positive when doing blob detection, otherwise
# you can set the sampleSize variable to 1 and not perform averages
#
# Author(s): Jason Jopp, <your name here>
# -----------------------------------------------------------------------------

import cv2 as cv, numpy as np, warnings

# Used set the low/high HSV colorspace values the detector searches between
pink_ball = {
    'hLow' : 150,      # Hue low
    'hHigh' : 190,     # Hue high
    'sLow' : 80,       # Saturation low
    'sHigh' : 255,     # Saturation high
    'vLow' : 0,        # Value low
    'vHigh' : 255      # Value high
}

pink_box = {
    'hLow' : 160,      # Hue low
    'hHigh' : 185,     # Hue high
    'sLow' : 50,       # Saturation low
    'sHigh' : 255,     # Saturation high
    'vLow' : 0,        # Value low
    'vHigh' : 255      # Value high
}

# Used for blob detection to find the green box
green_box = {
    'hLow' : 55,       # Hue low
    'hHigh' : 100,     # Hue high
    'sLow' : 20,      # Saturation low
    'sHigh' : 255,     # Saturation high
    'vLow' : 0,      # Value low
    'vHigh' : 255      # Value high
}

# Used for blob detection to find the blue bucket beacon
blue_bucket = {
    'hLow' : 75,       # Hue low
    'hHigh' : 110,     # Hue high
    'sLow' : 155,      # Saturation low
    'sHigh' : 255,     # Saturation high
    'vLow' : 110,      # Value low
    'vHigh' : 255      # Value high
}


class Detector:
    # Dictates sample size blobDetection uses when averaging coordinates & size
    sampleSize = 1
    
    def __init__(self,hsv_params):
        self.hsv_params = hsv_params
        self.create_detector()
        # Used to calculate the rolling average of blob coordinates
        self.coords = []
        self.avgXY = (0,0)
        # Used to calculate the rolling average of blob sizes
        self.sizes = []
        self.avgSize = 0.0
    
    def create_detector(self):
        """
        Creates a detector, which holds the parameters for blob detection.
        Detector should only need to be made once after params are set.
        """
        
        # Initializes thresholds for blob detection
        self.params = cv.SimpleBlobDetector_Params()

        # Sets blob detection parameters to given thresholds, blobs outside of
        # the given params will not be detected as blobs.
        self.params.minThreshold = self.hsv_params['vLow']
        self.params.maxThreshold = self.hsv_params['vHigh']

        self.params.minArea = 300            # Minimum area of blob in pixels
        self.params.maxArea = 999999999999        # Maximum area of blob in pixels
        self.params.minCircularity = 0.15     # Minimum blob circularity

        # The code prevents step size errors, step size must be less than than 
        # diff of vHigh and vLow, and step size cannot be zero. The larger the 
        # step size the less steps are taken during frame examination. There is
        # a notable increase in program speed by having less steps, with no 
        # apparent drop off in blob detection capability.
        stepSize = abs(self.hsv_params['vHigh']-self.hsv_params['vLow'])
        threshStep = 254
        if (threshStep >= stepSize):
            if stepSize < 2:
                self.params.thresholdStep = 1
            else:
                self.params.thresholdStep = stepSize - 1
        elif (threshStep > 0):
            self.params.thresholdStep = 254

        # Creates and returns the detector object based on the set parameters
        self.detector = cv.SimpleBlobDetector_create(self.params)

        # creating tuples for limits used to generate mask for each image frame
        self.lows = (self.hsv_params['hLow'],self.hsv_params['sLow'],self.hsv_params['vLow'])
        self.highs = (self.hsv_params['hHigh'],self.hsv_params['sHigh'],self.hsv_params['vHigh'])

    def blob_detector(self,frame):
        """
        Takes a BGR frame, converts it to HSV, creates a mask,
        then returns the coordinates and size of the blob found.
        """

        # Converts BGR to HSV frame, HSV is less sensitive to changes in light
        hsvFrame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        # Generates mask using hsvFrame, and HSV parameters
        mask = cv.inRange(hsvFrame,self.lows,self.highs)

        # Sets opening/closing of mask, look at "OpenCV Morphology" for more info
        kernel = np.ones((5,5), np.uint8)
        # Removes many false positives in blob detection
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel, iterations = 2)
        # Removes many false negatives in blob detection
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
        # Drawn rectangle allows blob detection when blob is on edge of frame
        mask = cv.rectangle(mask, (0,0), (639,479), (0,0,0), 1)
         
        # Detects blobs, creates frame for displaying blobs
        blobs = self.detector.detect(cv.bitwise_not(mask))
    
        # Calcs rolling average of blob coordinates/size, sampleSize
        # Only runs if there is a single blob detected in the frame examined
        if 0 < len(blobs) < 2:
            # Pulls xy coordinate and blob size from the first blob on the list.
            xCoord = round(blobs[0].pt[0])
            yCoord = round(blobs[0].pt[1])
            size = round(blobs[0].size)
        
            # Maintains sample size
            while (len(self.coords) >= Detector.sampleSize):
                del self.coords[0]
                del self.sizes[0]
            self.coords.append((xCoord,yCoord))
            self.sizes.append(size)

            print(f"Blob Coords: {xCoord},{yCoord}  Blob Size: {size}")
        
        # If more or less than one blob is detected, it starts filling coord/size
        # lists with nans, so that the rover isn't making decisions off of old data
        else:
            while (len(self.coords) >= Detector.sampleSize):
                del self.coords[0]
                del self.sizes[0]
            self.coords.append((np.nan,np.nan))
            self.sizes.append(np.nan)

        # Averages and rounds coordinates along 0 axis, ignoring NaNs
        # Warning is caught because anticipated warning with empty mean of all NaNs
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", category=RuntimeWarning)
            self.avgXY = np.round_(np.nanmean(self.coords, axis=0))
            self.avgSize = np.round_(np.nanmean(self.sizes, axis=0))

        return self.avgXY, self.avgSize
