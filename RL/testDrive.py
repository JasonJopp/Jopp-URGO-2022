# NOTE: The purpose of this program is to test the Sphero Rover's drive functionality.
# This program is not required for the machine learning research.

import os
import sys
import asyncio

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../sphero-sdk-raspberrypi-python/')))

from sphero_sdk import SpheroRvrObserver
from drive import driver

loop = asyncio.get_event_loop()

# Creates RVR object
rvr = SpheroRvrObserver()

if __name__ == '__main__':
    try:
        command = [rvr, 2, 1, 5, 64, "Soft Left"]
        loop.run_until_complete(
            driver(*command)
        )

    except KeyboardInterrupt:
        print('\nProgram terminated with keyboard interrupt.')

        loop.run_until_complete(
            rvr.close()
        )

    finally:
        pass
