import os
import sys
import time

#sys.path.append('../')
#sys.path.append('../..')
#sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../')))
sys.path.append('/home/pi/sphero-sdk-raspberrypi-python')

from sphero_sdk import SpheroRvrObserver

# Creates a rover object
rvr = SpheroRvrObserver()

def main():
    # Wakes up rvr to receive commands
    rvr.wake()
    
    # Gives rvr time to wake up
    time.sleep(2)
    
    # Resets rvr heading to zero
    rvr.drive_control.reset_heading()
    
    rvr.drive_control.drive_forward_seconds(
        speed=10,
        heading=0,
        time_to_drive=1
    )

if __name__ == '__main__':
    try:
        # Stuff we want to do, calling main function
        main()
    
    except KeyboardInterrupt:
        # What happens if there's a keyboard interrupt (ctrl+c)
        print('\nProgram terminated with keyboard interrupt')
    
    finally:
        rvr.close()

