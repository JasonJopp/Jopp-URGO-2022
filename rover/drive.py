import os
import sys
import time

sys.path.append('../')
sys.path.append('../..')
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../')))

from sphero_sdk import SpheroRvrObserver
from sphero_sdk import RawMotorModesEnum
rvr = SpheroRvrObserver()

# in range from 0 to 255
MAX_SPEED = 255

# default speed
SPEED = 140

#asleep = True

def drive(dir, timeout):

    #if asleep: 
     #   rvr.wake()
      #  asleep = False 

    # Give RVR time to wake up
    #time.sleep(1)
    rvr.reset_yaw()
    
    rvr.set_custom_control_system_timeout(command_timeout=timeout)  
    
    
    if (dir=="right"):
        rvr.raw_motors(
            left_mode=RawMotorModesEnum.forward.value,   
            left_duty_cycle = SPEED,
            right_mode=RawMotorModesEnum.reverse.value,
            right_duty_cycle= SPEED
        )
    elif(dir == "left"):
        rvr.raw_motors(
            left_mode=RawMotorModesEnum.reverse.value,   
            left_duty_cycle = SPEED,
            right_mode=RawMotorModesEnum.forward.value,
            right_duty_cycle = SPEED
        )
    elif(dir == "reverse"):
        rvr.raw_motors(
            left_mode=RawMotorModesEnum.reverse.value,   
            left_duty_cycle = SPEED,
            right_mode=RawMotorModesEnum.reverse.value,
            right_duty_cycle = SPEED
        )
    else:
        print("Test 1")
        rvr.raw_motors(
            left_mode=RawMotorModesEnum.forward.value,   
            left_duty_cycle = SPEED,
            right_mode=RawMotorModesEnum.forward.value,
            right_duty_cycle = SPEED
        )
        print("Test 2")
        

        
def drive_second():
    #this is a simple code to run the wheels of the RVR forward
    print("Test 1")
    rvr.wake()
    
    # Give RVR time to wake up
    #time.sleep(1)
    print("Test 2")
    rvr.reset_yaw()
    print("Test 3")
    rvr.set_custom_control_system_timeout(command_timeout=535)  
    print("Test 4")
    rvr.raw_motors(
            # could be adjusted to move in the reverse direction by changing forward to reverse
            left_mode=RawMotorModesEnum.reverse.value,   
            left_duty_cycle = 140,  # Valid duty cycle range is 0-255
            right_mode=RawMotorModesEnum.forward.value,
            right_duty_cycle = 140  # Valid duty cycle range is 0-255
        )
    print("Test 5")
    
            
if __name__ == '__main__':
    print("Test 0")
    drive_second()
     
