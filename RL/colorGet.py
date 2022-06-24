from threading import Thread

class ColorGet:
    """
    Class continually gets color information from the color sensor under the
    Sphero RVR. Which is used to indicate if the RVR has gone out of bounds,
    which is marked with colored tape on the ground.
    """
    def __init__(self):
        pass

    def start(self):
        """Starts the colorCapture thread."""
        Thread(target=self.get, args=()).start()
        self.stopped = False
    
    def get(self):
        """Gets the most recently seen color from colorCapture"""
        while not self.stopped:
            pass #TODO: Need to add loop for getting the most recent color.

    def stop(self):
        """Stops the ColorCapture thread."""
        self.stopped = True
        # TODO: Add any other closing commands that the RVR requires here.