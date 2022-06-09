from threading import Thread
import cv2 as cv

class VideoShow:
    """Class continually shows frame using a dedicated thread"""

    def __init__(self, frame = None):
        self.frame = frame
        self.stopped = False

    def start(self):
        Thread(target = self.show, args=()).start()
        return self
    
    def show(self):
        while not self.stopped:
            cv.imshow('Video', self.frame)
            if cv.waitKey(1) == ord('q'):
                self.stopped = True
    
    def stop(self):
        self.stopped = True
