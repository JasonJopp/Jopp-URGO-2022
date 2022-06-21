import servoingEnvironment
import numpy as np
import cv2 as cv
import time
from videoGet import VideoGet

env = servoingEnvironment.ServoingEnvironment()            

#Initialize Q-table with all zeros
Q = np.zeros([env.numStates,env.numActions])

# Set learning parameters
learningRate = .8   
gamma = .95 # Gamma setting for updating the Q-table  
numEpisodes = 2

#create lists to contain total rewards and steps per episode
rList = []

# Creates VideoCapture thread
videoGetter = VideoGet()
videoGetter.start()

def getFrame():
    """
    Dedicated thread for grabbing video frames with videoGet obj.
    Main thread shows video frames.
    """
    frame = videoGetter.frame
    cv.imshow('frame', frame)
    return frame

def trainerFunc():    
    for i in range(numEpisodes):
        
        # Sets flag for initial setup to True, for use in each episode
        initalSetup = True
        stepNumber = 0

        #The Q-Table learning algorithm
        while stepNumber < 99: # Max amount of steps allowed before episode times out and fails
            if (cv.waitKey(1) == ord('q') or videoGetter.stopped):
                videoGetter.stop()

            # Runs initial setup once for each episode
            if initalSetup == True:
                print("Beginning episode",i,"...")
                currentState = env.reset(getFrame()) # Defines initial state of the system
                #Reset environment and get initial observation
                rAll = 0 # Quantifies rewards over time
                winStatus = False # Defines if episode succeeded or failed
                initalSetup = False # Makes it so initial step only runs on first iteration

            print("Starting trial",stepNumber,"in state",currentState,"...")
            stepNumber+=1
            #Choose an action by greedily (with noise) picking from Q table

            # random noise added to each column element in row "s"
            # then the INDEX of the maximum of those is selected
            # np.argmax(Q[s,:] -- Searches table for action that is most likely to succeed given the state
            # np.random.randn(1,env.numActions)*(1./(i+1))) -- creates random noise in action selection that becomes less random as episodes continue
            # Selects an action to take
            action = np.argmax(Q[currentState,:] + np.random.randn(1,env.numActions)*(1./(i+1)))
            
            # Displays selected action
            print("Selected action",action)

            # Get new state and reward from environment
            # Takes new step using the action 'a', gets new state, reward (1 or 0), and whether or not the episode succeeded (True/False)
            # newState,reward,winStatus = asyncio.run(asyncio.gather(env.step(action)))
            newState,reward,winState = env.step(action, getFrame())
            print("Action performed. In state",newState,". reward:",reward)

            if (reward==1):
                # SUCCESS. update the table and start again.
                print("Goal Achieved!!")

            print("\nNEW STATE",newState)

            #Update Q-Table with new knowledge
            Q[currentState,action] = Q[currentState,action] + learningRate*(reward + gamma*np.max(Q[newState,:]) - Q[currentState,action])
            rAll += reward
            currentState = newState

            # if success or fail, start a new episode
            if winStatus == True:
                break

        rList.append(rAll)

def main():
    trainerFunc()

main()
