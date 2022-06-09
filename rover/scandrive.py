import sys
import os
import numpy as np
import cv2 as cv
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../')))

import asyncio

from helper_keyboard_input import KeyboardHelper
from sphero_sdk import SerialAsyncDal
from sphero_sdk import SpheroRvrAsync

# initialize global variables
key_helper = KeyboardHelper()
current_key_code = -1
driving_keys = [119, 97, 115, 100, 32]
speed = 0
heading = 0
flags = 0
changes = True

loop = asyncio.get_event_loop()
rvr = SpheroRvrAsync(
    dal=SerialAsyncDal(
        loop
    )
)

def change_detector(x):
	global changes
	changes = True

def keycode_callback(keycode):
    global current_key_code
    current_key_code = keycode
    print("Key code updated: ", str(current_key_code))

async def main():
    """
    Runs the main control loop for this demo.  Uses the KeyboardHelper class to read a keypress from the terminal.

    W - Go forward.  Press multiple times to increase speed.
    A - Decrease heading by -10 degrees with each key press.
    S - Go reverse. Press multiple times to increase speed.
    D - Increase heading by +10 degrees with each key press.
    Spacebar - Reset speed and flags to 0. RVR will coast to a stop

    """
    global current_key_code
    global speed
    global heading
    global flags
    global changes

    # Array is used to calculate the rolling average of coordinates
    coords = []
    avgXY = (0,0)

    # Initializes thresholds for blob detection
    params = cv.SimpleBlobDetector_Params()
	
	# Selects camera to use
    capture = cv.VideoCapture(0, cv.CAP_ANY)
	
    # Checks if camera is found
    if not capture.isOpened():
        print("Cannot find camera")
        exit()

    # Dictates the size of the window created, based on array
    img = np.ones((1,509,1), np.uint8)

    cv.namedWindow('Settings')

    # Creates the different trackbars for blob detection thresholds
    # <bar name>, <win location>, <init val>, <max val>, <rtrn bar val>
    cv.createTrackbar('H Low', 'Settings', 0, 255, change_detector)
    cv.createTrackbar('H High', 'Settings', 255, 255, change_detector)
    cv.createTrackbar('S Low', 'Settings', 50, 255, change_detector)
    cv.createTrackbar('S High', 'Settings', 255, 255, change_detector)
    cv.createTrackbar('V Low', 'Settings', 0, 255, change_detector)
    cv.createTrackbar('V High', 'Settings', 255, 255, change_detector)
    cv.createTrackbar('Min Area', 'Settings', 300, 1000, change_detector)
    cv.createTrackbar('Max Area', 'Settings', 200000, 200000, change_detector)
    cv.createTrackbar('Min Circ', 'Settings', 150, 1000, change_detector)
    cv.createTrackbar('Max Circ', 'Settings', 1000, 1000, change_detector)
    cv.createTrackbar('Min Blob Dist', 'Settings', 0, 1000, change_detector)
    cv.createTrackbar('Thresh Step', 'Settings', 254, 254, change_detector)
    cv.createTrackbar('Window Size', 'Settings', 10, 50, change_detector)

    # Flag for initial setup of while loop for windows
    # This flag is for things I only want to run once in the loop
    init_window = True

    # Rover wake up routine
    await rvr.wake()
    await rvr.reset_yaw()

    while True:
        cv.imshow("Settings",img)

        # Captures each frame, converts from BGR to HSV colorcode
        ret, frame = capture.read()

        # Changes color format from BGR to HSV
        hsvFrame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        # Only creates a new detector if changes were made to the params
        if changes:
            changes = False
            hLow = cv.getTrackbarPos('H Low', 'Settings')
            hHigh = cv.getTrackbarPos('H High', 'Settings')
            sLow = cv.getTrackbarPos('S Low', 'Settings')
            sHigh = cv.getTrackbarPos('S High', 'Settings')
            vLow = cv.getTrackbarPos('V Low', 'Settings')
            vHigh = cv.getTrackbarPos('V High', 'Settings')
            windowSize = cv.getTrackbarPos('Window Size', 'Settings')
            if windowSize == 0:
                windowSize = 1

            # Sets blob detection parameters to trackbar
            params.minThreshold = vLow
            params.maxThreshold = vHigh
            params.minArea = cv.getTrackbarPos('Min Area', 'Settings')
            params.maxArea = cv.getTrackbarPos('Max Area', 'Settings')
            params.minCircularity = cv.getTrackbarPos('Min Circ', 'Settings')/1000
            params.maxCircularity = cv.getTrackbarPos('Max Circ', 'Settings')/1000
            params.minDistBetweenBlobs = cv.getTrackbarPos('Min Blob Dist', 'Settings')
            # This prevents errors with threshold step size
            # Step size must be less than than diff of vHigh and vLow
            # Step size also cannot be zero
            stepSize = abs(vHigh-vLow)
            if (cv.getTrackbarPos('Thresh Step', 'Settings') >= stepSize):
                if stepSize < 2:
                    params.thresholdStep = 1
                else:
                    params.thresholdStep = stepSize - 1
            elif (cv.getTrackbarPos('Thresh Step', 'Settings') > 0):
                params.thresholdStep = cv.getTrackbarPos('Thresh Step', 'Settings')
            
            # Creates the detector object based on the set params
            detector = cv.SimpleBlobDetector_create(params)

		# Creates mask for hsvFrame, large CPU performance sink
        mask = cv.inRange(hsvFrame, (hLow, sLow, vLow), (hHigh, sHigh, vHigh))

        # Sets "opening"/"closing" of mask (Morphology)
        kernel = np.ones((5,5), np.uint8)
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel, iterations = 2) # Removes false positives
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel) # Removes false negatives

        # Detects blobs, creates frame for displaying blobs
        blobs = detector.detect(cv.bitwise_not(mask))
        hsvBlobs = cv.drawKeypoints(frame, blobs, np.array([]), (0,255,0), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # Function called when mouse event happens in frame
        #cv.setMouseCallback('frame', click_event)

        # Waits for 'q' to close program
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

        # Creates rolling average of coordinates, dependant on windows_size
        if 0 < len(blobs) < 2:	
            xCoord = round(blobs[-1].pt[0])
            yCoord = round(blobs[-1].pt[1])
            while (len(coords) >= windowSize): # Window size sets list size to be averaged into coordinate point
                del coords[0]
            coords.append((xCoord,yCoord))
            avgXY = np.round_(np.mean(coords, axis=0)) # Averages and rounds coordinates along 0 axis

        hsvBlobs = cv.circle(hsvBlobs, (int(avgXY[0]),int(avgXY[1])), 20, (255,0,0), 2)
        hsvBlobs = cv.circle(hsvBlobs, (320,210), 5, (0,0,255), 3)

        # Displays different frames until 'q' is pressed
        cv.imshow('mask',mask)
        cv.imshow('hsvBlobs', hsvBlobs)

        # This section only runs things in this loop once, for setup
        if (init_window == True):
            # Moves windows prevent stacking
            cv.moveWindow('Settings', 0, 40)
            cv.moveWindow('mask', 512, 40)
            cv.moveWindow('hsvBlobs', 1155, 40)
            init_window = False

        # Rover controls

        if current_key_code == 119:  # W
            # if previously going reverse, reset speed back to 64
            if flags == 1:
                speed = 32
            else:
                # else increase speed
                speed += 32
            # go forward
            flags = 0
        elif current_key_code == 97:  # A
            heading -= 10
        elif current_key_code == 115:  # S
            # if previously going forward, reset speed back to 64
            if flags == 0:
                speed = 32
            else:
                # else increase speed
                speed += 32
            # go reverse
            flags = 1
        elif current_key_code == 100:  # D
            heading += 10
        elif current_key_code == 32:  # SPACE
            # reset speed and flags, but don't modify heading.
            speed = 0
            flags = 0

        # check the speed value, and wrap as necessary.
        if speed > 255:
            speed = 255
        elif speed < -255:
            speed = -255

        # check the heading value, and wrap as necessary.
        if heading > 359:
            heading = heading - 359
        elif heading < 0:
            heading = 359 + heading

        # reset the key code every loop
        current_key_code = -1

        # issue the driving command
        await rvr.drive_with_heading(speed, heading, flags)

        # sleep the infinite loop for a 10th of a second to avoid flooding the serial port.
        await asyncio.sleep(0.1)
    
    # Releases camera frame capture
    capture.release()

    # Closes all GUI elements
    cv.destroyAllWindows


def run_loop():
    global loop
    global key_helper
    key_helper.set_callback(keycode_callback)
    loop.run_until_complete(
        asyncio.gather(
            main()
        )
    )
    


if __name__ == "__main__":
    loop.run_in_executor(None, key_helper.get_key_continuous)
    try:
        run_loop()
    except KeyboardInterrupt:
        print("Keyboard Interrupt...")
        key_helper.end_get_key_continuous()
    finally:
        print("Press any key to exit.")
        exit(1)
