# -----------------------------------------------------------------------------
# Code Purpose: 
# 
# 
# -----------------------------------------------------------------------------


from threading import Thread

from sphero_sdk import SpheroRvrObserver
from sphero_sdk import RvrStreamingServices

class ColorGet:
    """
    Class continually gets color information from the color sensor under the
    Sphero RVR. Which is used to indicate if the RVR has gone out of bounds,
    which is marked with colored tape on the ground.
    """
    def __init__(self, rvr):
        rvr.enable_color_detection(is_enabled=True)

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
        rvr.sensor_control.clear()
        # TODO: Add any other closing commands that the RVR requires here.