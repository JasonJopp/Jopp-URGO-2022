import numpy as np
import cv2 as cv
import asyncio

class FinderDriver:

    def __init__(self):
        # Initializes blob thresholds
        self.params = cv.SimpleBlobDetector_Params()

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

    async def getVideoFrame(self):
        # Selects camera to use
        capture = cv.VideoCapture(0, cv.CAP_ANY)
        
        # Checks if camera is found
        if not capture.isOpened():
            print("ERROR: Could not open video capture.")
            exit()

        while True:
            # Captures each frame, converts from BGR to HSV colorcode
            ret, frame = capture.read()
            hsvFrame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

            cv.imshow('frame',frame)

            # Displays windows for given milliseconds
            cv.waitKey(1)

    async def main(self):
        # Can add more async functions here to run them
        await asyncio.gather(driver.getVideoFrame())

if __name__ == "__main__":    
    driver = FinderDriver()
    asyncio.run(driver.main())
    
     
