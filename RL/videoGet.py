# -----------------------------------------------------------------------------
# Code Purpose: To run video capture in a seperate thread, so that the frame
# provided to the rest of the program is as recent as possible. If the capture
# is not performed in a separate thread, the frames will quickly become old
# and the RVR will take actions based on what it saw in the past, not what it
# sees in the near-present.
#
# NOTE: Using imshow to display the frame likely won't show the current frame 
# being analyzed by the servoring environment. It lags behind by a few frames.
# 
# Author(s): Jason Jopp, <your name here>
# -----------------------------------------------------------------------------

from threading import Thread
import cv2 as cv

class VideoGet:
    """
    Class that continually gets frames from a 
    VideoCapture obj with a dedicated thread.
    """
    def __init__(self, src=0):
        self.capture = cv.VideoCapture(src)
        self.capture.set(cv.CAP_PROP_BUFFERSIZE, 1)

        if not self.capture.isOpened():
            print("Could not find camera.")
            exit()
        self.ret, self.frame = self.capture.read()
        self.stopped = False

    
    def start(self):
        """Starts the VideoCapture thread."""
        Thread(target=self.get, args=()).start()
        self.stopped = False

    def get(self):
        """Gets the most recent frame from VideoCapture."""
        while not self.stopped:
            if not self.ret:
                self.stop()
            else:
                (self.ret, self.frame) = self.capture.read()
        
    
    def stop(self):
        """Stops the VideoCapture thread."""
        self.stopped = True
        self.capture.release()
        cv.destroyAllWindows()
