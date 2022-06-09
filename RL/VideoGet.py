from threading import Thread
import cv2 as cv

class VideoGet:
    """Dedicated thread for getting frames from VideoCapture object."""

    def __init__(self, src = 0):
        # Opens video capture, verifies capture is open
        self.capture = cv.VideoCapture(src)
        if not self.capture.isOpened():
            print("Cannot find camera")
            exit()
        
        # Gets image from capture
        (self.ret, self.frame) = self.capture.read()
        self.stopped = False
    
    def start(self):
        """Creates and starts a thread for VideoCapture."""
        Thread(target = self.get, args = ()).start()
        return self
    
    def get(self):
        while not self.stopped:
            if not self.ret:
                self.stop()
            else:
                (self.ret, self.frame) = self.capture.read()
                if (cv.waitKey(1) == ord('q')):
                    break
    
    def stop(self):
        self.stopped = True

##################################### Use the below in the main file
"""
import cv2 as cv
from VideoGet import VideoGet

def threadVideoGet(source = 0):

    #Dedicated thread for grabbing video frames with VideoGet object.
    #Main thread shows video frames.


    video_getter = VideoGet(source).start()

    while True:
        if (cv.waitKey(1) == ord('q')) or video_getter.stopped:
            video_getter.stop()
            break
        
        frame = video_getter.frame
        cv.imshow("Video", frame)

"""
