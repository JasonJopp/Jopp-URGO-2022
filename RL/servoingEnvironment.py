import os, sys, asyncio, numpy as np, cv2 as cv, random
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../sphero-sdk-raspberrypi-python/')))
from sphero_sdk import SpheroRvrObserver
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
        random.uniform(.1,.5), 180, "Restart", [255,0,0]]

        return restartDrive
    
    def reset(self, videoGetter):
        # Put the robot in a starting position.
        print("RVR being reset")
        restartDriveCommand = self.randomRestart(self.rvr)
        asyncio.run(driver(*restartDriveCommand))
        return self.get_state(videoGetter)
    
    def get_state(self, videoGetter):
        """Gets a video frame, searches for blobs in the frame using the detector,
        """
        frame = videoGetter.frame
        avgXY, avgSize = blobDetector(frame, self.detector)
        if not np.isnan(avgXY[0]):
            frame = cv.circle(frame, (int(avgXY[0]), int(avgXY[1])), int(avgSize//2), (0,255,0), 3)
        cv.imshow('Frame', frame)
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

