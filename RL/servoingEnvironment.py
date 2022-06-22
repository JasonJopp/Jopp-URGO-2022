import os, sys, asyncio, numpy as np, cv2 as cv, random
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../sphero-sdk-raspberrypi-python/')))
from sphero_sdk import SpheroRvrObserver, Colors
from videoGet import VideoGet

from drive import driver
from detector import createDetector, blobDetector

class ServoingEnvironment:

    def __init__(self) -> None:
        
        # Creates RVR object
        self.rvr = SpheroRvrObserver()

        # Lists out actions that the rover can take.
        # NOTE: Legend: [rvrObject, L-trk drive mode, R-trk drive mode,.. 
        # ..drive time (seconds), speed, command name, led color code]
        # Drive Modes: 0 - Stop, 1 - Forward, 2 - Reverse
        self.actions = []
        self.actions.append([self.rvr, 1, 2, .5, 180, "Hard Right"])
        self.actions.append([self.rvr, 1, 2, .2, 180, "Right"])
        self.actions.append([self.rvr, 1, 2, .1, 180, "Soft Right"])
        #self.actions.append([self.rvr, 1, 1, 1, 180, "Forward"])
        #self.actions.append([self.rvr, 2, 2, 1, 180, "Reverse"])
        self.actions.append([self.rvr, 2, 1, .1, 180, "Soft Left"])
        self.actions.append([self.rvr, 2, 1, .2, 180, "Left"])
        self.actions.append([self.rvr, 2, 1, .5, 180, "Hard Left"])

        # Gets number of possible actions
        self.numActions = len(self.actions)
        self.image_divisions = 7
        # Additional state is for being off-camera
        self.numStates = self.image_divisions + 1
        self.image_width = 640

        # signifies blob is not in the visual field
        self.no_blob = self.numStates - 1

        # Gets middle slice of image divisions for reward state
        self.reward_state = ((self.image_divisions)//2)
        self.current_state = self.no_blob

        # Creates a detector for blob detection, obj holds blob params
        self.detector = createDetector()

    def randomRestart(self, rvr):
        # Generates a random-ish drive command for restarting the test
        restartLeftTrack = random.randint(1,2)
        if restartLeftTrack == 1:
            restartRightTrack = 2
        else:
            restartRightTrack = 1
        restartDrive = [rvr, restartLeftTrack, restartRightTrack, 
        random.uniform(.1,.5), 180, "Restart", [255,0,255]]

        return restartDrive
    
    def reset(self, videoGetter):
        # Put the robot in a starting position.
        print("RVR being reset")
        state = self.no_blob
        while state == self.no_blob:
            restartDriveCommand = self.randomRestart(self.rvr)
            asyncio.run(driver(*restartDriveCommand))
            state = self.get_state(videoGetter)
        
        return state
    
    def get_state(self, videoGetter):
        """Gets a video frame, searches for blobs in the frame using the detector,
        """
        # Gets frame from videoGetter obj
        frame = videoGetter.frame
        avgXY, avgSize = blobDetector(frame, self.detector)
        
        # Draws circle on blob location, if available
        if not np.isnan(avgXY[0]):
            frame = cv.circle(frame, (int(avgXY[0]), int(avgXY[1])), int(avgSize//2), (0,255,0), 3)
        
        # Displays the frame in a new window
        cv.imshow('Frame', frame)

        # Returns state dependent on if there is a blob on frame or not
        if not np.isnan(avgXY[0]) and not np.isnan(avgSize):
            state = int(avgXY[0]*self.image_divisions/self.image_width)
        else:
            state = self.no_blob
        return state

    def step(self, action, videoGetter):
        # Send action command to robot and get next state.
        driveParams = self.actions[action]
        asyncio.run(driver(*driveParams))
        print(f"Driving: {driveParams[-1]}")
        new_state = self.get_state(videoGetter)

        reward = 0
        completeStatus = False

        if (new_state == self.reward_state):
            reward = 1
            completeStatus = True
            # reached goal. done, leds set to green
            self.rvr.led_control.set_all_leds_rgb(red=0, green=255, blue=0)
        elif (new_state == self.no_blob):
            reward = 0
            completeStatus = True
            # failed. done, leds set to red
            self.rvr.led_control.set_all_leds_rgb(red=255, green=0, blue=0)
        else:
            reward = 0
            # Not done, still searching, leds set to orange
            self.rvr.led_control.set_all_leds_rgb(red=255, green=165, blue=0)
        return new_state, reward, completeStatus

