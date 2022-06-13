import cv2 as cv
import signal
import numpy as np

from VideoGet import VideoGet

class BlobFinderDriver:
    """Returns blob coordinates of given frame."""
    
    def __init__(self):
        self.exit_event = threading.Event() # Used to stop threads
        self.video_getter = VideoGet(0).start() # Starts video capture
        self.params = cv.SimpleBlobDetector_Params() # Initializes blob thresholds
        
        # Sets Hue thresholds (Color)
        self.hLow = 150
        self.hHigh = 190

        # Sest Saturation thresholds (color boldness/deepness)
        self.sLow = 70
        self.sHigh = 255

        # Changes Value threshold (Light)
        self.vLow = 0
        self.vHigh = 255

        # Array is used to calculate the rolling average of coordinates
        self.coords = []

        # Sets array length for rolling average of coordinates returned
        self.windowSize = 5
        self.avgXY = [0,0]

    def threadVideoGet(self):
        """
        Dedicated thread for grabbing a video frame with VideoGet object.
        Converts frame from BGR to HSV for further analysis.
        """
        frame = self.video_getter.frame
        hsvFrame = cv.cvtColor(self.video_getter.frame, cv.COLOR_BGR2HSV)
        return frame, hsvFrame


    def blobDetectorCreator(self):
        """Uses threshold parameters to create a blob detector."""

        # Changes Value threshold (Light)
        self.params.minThreshold = 0
        self.params.maxThreshold = 255

        # Sets Area threshold
        self.params.filterByArea = True
        self.params.minArea = 100
        self.params.maxArea = 200000

        # Sets circularity threshold, square is ~.78
        self.params.filterByCircularity = True
        self.params.minCircularity = 0.3

        # Creates and returns the detector used for mask creation
        detector = cv.SimpleBlobDetector_create(self.params)
        return detector


    def blobFinder(self, frame, detector):
        # Creates mask for hsvFrame, large CPU performance sink
        mask = cv.inRange(frame, (self.hLow, self.sLow, self.vLow), (self.hHigh, self.sHigh, self.vHigh))
        
        # Sets "opening"/"closing" of mask (Morphology)
        kernel = np.ones((5,5), np.uint8)
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel, iterations = 2) # Removes false positives
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel) # Removes false negatives

        # Finds blob coordinates using mask
        blobCoords = detector.detect(cv.bitwise_not(mask))

        # Creates rolling average of coordinates, dependant on windows_size
        if 0 < len(blobCoords) < 2:	
            xCoord = round(blobCoords[-1].pt[0])
            yCoord = round(blobCoords[-1].pt[1])
            while (len(self.coords) >= self.windowSize): # Change window size here to make examined array larger
                del self.coords[0] # Deletes oldest coordinate
            self.coords.append((xCoord,yCoord)) # Adds newest coordinate to window
        else: # If no blob is found, adds nan to the window.
            while (len(self.coords) >= self.windowSize):
                del self.coords[0]
            self.coords.append((np.nan,np.nan))
        
        self.avgXY = np.round_(np.nanmean(self.coords, axis=0)) # Averages and rounds window coords, ignores nans
        return mask

    def exit_handler(self, signum, frame):
        """Used to shut down the program."""
        # Stops the video_getter
        self.video_getter.stop
        exit(0)
    
    def main(self):
        
        signal.signal(signal.SIGINT, BlobFinderDriver.exit_handler)
        detector = self.blobDetectorCreator()
        
        while True:
            frame, hsvFrame = self.threadVideoGet()
            mask = self.blobFinder(hsvFrame, detector)
            if not (np.isnan(self.avgXY[0])):
                frame = cv.circle(frame, (int(self.avgXY[0]),int(self.avgXY[1])), 20, (255,0,0), 2)
                # Returns blob x,y coordinates as two seperate variables
                return self.avgXY[0], self.avgXY[1]

    