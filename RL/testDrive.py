import os
import sys
import asyncio

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../sphero-sdk-raspberrypi-python/')))

from sphero_sdk import SpheroRvrAsync, SerialAsyncDal
from drive import driver

loop = asyncio.get_event_loop()

# Creates RVR object
rvr = SpheroRvrAsync(
    dal=SerialAsyncDal(
        loop
    )
)

if __name__ == '__main__':
    try:
        command = [rvr, 1, 1, 5, 64, "Soft Left"]
        loop.run_until_complete(
            driver(*command)
        )

    except KeyboardInterrupt:
        print('\nProgram terminated with keyboard interrupt.')

        loop.run_until_complete(
            rvr.close()
        )

    finally:
        if loop.is_running():
            loop.close()
