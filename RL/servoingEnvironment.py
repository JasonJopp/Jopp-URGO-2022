import os, sys, time, random, asyncio, numpy as np, cv2 as cv
from time import sleep
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../sphero-sdk-raspberrypi-python/')))
from sphero_sdk import SpheroRvrAsync, SerialAsyncDal

from drive import driver
from detector import createDetector, blobDetector

class ServoingEnvironment:

    def __init__(self) -> None:
        loop = asyncio.get_event_loop()

        # Creates RVR object
        rvr = SpheroRvrAsync(
            dal=SerialAsyncDal(
                loop
            )
        )

        # Lists out actions that the rover can take.
        # NOTE: Legend: [rvrObject, L-trk drive mode, R-trk drive mode,.. 
        # ..drive time (seconds), speed, command name]
        # Drive Modes: 0 - Stop, 1 - Forward, 2 - Reverse
        self.actions = []
        self.actions.append([rvr, 1, 2, 2, 64, "Hard Right"])
        self.actions.append([rvr, 1, 2, 1, 64, "Right"])
        self.actions.append([rvr, 1, 2, .5, 64, "Soft Right"])
        self.actions.append([rvr, 1, 1, 1, 64, "Forward"])
        self.actions.append([rvr, 2, 2, 1, 64, "Reverse"])
        self.actions.append([rvr, 2, 1, .5, 64, "Soft Left"])
        self.actions.append([rvr, 2, 1, 1, 64, "Left"])
        self.actions.append([rvr, 2, 1, 2, 64, "Hard Left"])

        # Gets number of possible actions
        self.num_of_actions = len(self.actions)
        self.image_divisions = 11
        self.num_of_states = self.image_divisions + 1
        self.image_width = 640

        # signifies blob is not in the visual field
        self.no_blob = self.num_of_states - 1

        self.reward_state = (self.image_divisions)//2    # in the middle
        self.current_state = self.no_blob

        # Sets up camera for video capture
        self.capture = cv.VideoCapture(0)
        if not self.capture.isOpened():
            print("Could not find camera.")
            exit()
        
        # Creates a detector for blob detection, obj holds blob params
        self.detector = createDetector()
    
    def reset(self):
        # Put the robot in a starting position.
        print("RVR being reset")
        #drive.drive(["right"],221)
        return self.get_state()
    
    def get_state(self):
        ret, frame = self.capture.read()
        avgXY = blobDetector(frame, self.detector)
        if not np.isnan(avgXY[0]):
            state = int(avgXY[0]*self.image_divisions/640)
        else:
            state = self.no_blob
        return state

    def step(self, action):
        # Send action command to robot and get next state.
        driveParams = self.actions[action]
        driver(*driveParams) # Runs drive command
        print(f"Driving with command: {driveParams[-1]}")
        new_state = self.get_state()
        reward = 0
        done = False

        if (new_state == self.reward_state):
            reward = 1
            done = True    # reached goal. done
        elif (new_state == self.no_blob):
            reward = 0
            done = True    # failed. done.
        else:
            reward = 0
        return new_state, reward, done
