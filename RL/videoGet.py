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
