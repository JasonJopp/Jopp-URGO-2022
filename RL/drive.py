from threading import Thread
import sys
import os
import asyncio

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../sphero-sdk-raspberrypi-python')))

from sphero_sdk import SpheroRvrAsync
from sphero_sdk import SerialAsyncDal
from sphero_sdk import RawMotorModesEnum

class DriveHandler:
    
    def __init__(self):
        # initialize global variables
        self.speed = 0
        self.heading = 0
        self.flags = 0

        self.loop = asyncio.get_event_loop()
        self.rvr = SpheroRvrAsync(
            dal=SerialAsyncDal(
                self.loop
            )
        )
        # Creates flag used for stopping program
        self.stopped = False
        
        

    def start(self):
        """Creates and starts a thread for DriveHandler."""
        Thread(target = self.drive, args = ()).start()
        return self
    
    async def drive(self, direction):
        """Accepts drive commands in a loop."""
        # RVR wake up routine
        
        self.rvr.wake()
        asyncio.sleep(2)
        self.rvr.reset_yaw()

        if direction == 'forward':
            self.rvr.raw_motors(
                left_mode=RawMotorModesEnum.forward.value,
                left_duty_cycle=128,  # Valid duty cycle range is 0-255
                right_mode=RawMotorModesEnum.forward.value,
                right_duty_cycle=128  # Valid duty cycle range is 0-255
            )

    
    def stop(self):
        self.stopped = True