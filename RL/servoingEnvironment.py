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
        self.actions.append([self.rvr, 1, 2, .15, 180, "Hard Right"])
        self.actions.append([self.rvr, 1, 2, .1, 180, "Right"])
        self.actions.append([self.rvr, 1, 2, .05, 180, "Soft Right"])
        #self.actions.append([self.rvr, 1, 1, 1, 180, "Forward"])
        #self.actions.append([self.rvr, 2, 2, 1, 180, "Reverse"])
        self.actions.append([self.rvr, 2, 1, .05, 180, "Soft Left"])
        self.actions.append([self.rvr, 2, 1, .1, 180, "Left"])
        self.actions.append([self.rvr, 2, 1, .15, 180, "Hard Left"])

        # Gets number of possible actions
        self.numActions = len(self.actions)
        self.image_divisions = 7
        
        # Image width of video frame, can be found using OpenCV
        self.image_width = 640

        # Used for returning the blob width state, which estimates how close a blob is
        self.blobSizeStateDict = {20:0, 40:1, 60:2, 80:3, 90:4, 999999:5}

        # Calculates total possible states, additional state is for no blob detected
        self.numStates = (self.image_divisions * len(self.blobSizeStateDict)) + 1

        # signifies blob is not in the visual field
        self.no_blob = self.numStates - 1

        # Gets middle slice of image divisions for reward state
        self.reward_state = (((self.image_divisions)//2) * len(self.blobSizeStateDict)) + max(self.blobSizeStateDict.values())
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

        return restartDrive, restartLeftTrack, restartRightTrack
    
    def reset(self, videoGetter):
        """This has the RVR place itself into a somewhat-random starting position by rotating"""
        print("RVR being reset")
        state = self.no_blob
        restartDriveCommand, restartLeftTrack, restartRightTrack = self.randomRestart(self.rvr)
        asyncio.run(driver(*restartDriveCommand))
        
        # The RVR will rotate until it sees a blob. (Search mode)
        while state == self.no_blob:
            state = self.get_state(videoGetter)
            if state != self.no_blob:
                break
            asyncio.run(driver(*[self.rvr, restartLeftTrack, restartRightTrack, .2, 180, "Scanning"]))
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
            # Gets the state of the x-coordinate of the blob
            blobXLocationState = int(avgXY[0]*self.image_divisions/self.image_width)
            # Gets the percentage size (blobRatio) of the blob, compared to the frame size (width)
            blobRatio = int((avgSize/self.image_width)*100)
            # Checks dictionary to assign blobSizeState to a number based on the blobRatio
            for size in self.blobSizeStateDict:
                if blobRatio < size:
                    blobSizeState = self.blobSizeStateDict[size]
                    break
            # Combines the blob x-coordinate state and blob size into a single state
            state = (blobXLocationState * len(self.blobSizeStateDict)) + blobSizeState
            self.rvr.led_control.set_all_leds_rgb(red=255, green=165, blue=0)
        else:
            state = self.no_blob
        return state

    def step(self, action, videoGetter):
        # Send action command to robot and get next state.
        driveParams = self.actions[action]
        asyncio.run(driver(*driveParams))
        print(f"Driving: {driveParams[-1]}")
        print(f"Reward State: {self.reward_state}")
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

