import servoingEnvironment
import numpy as np
import cv2 as cv
from videoGet import VideoGet

env = servoingEnvironment.ServoingEnvironment()            

#Initialize Q-table with all zeros
Q = np.zeros([env.numStates,env.numActions])

# Set learning parameters
learningRate = .8   
gamma = .95 # Gamma setting for updating the Q-table  
numEpisodes = 100

#create lists to contain total rewards and steps per episode
rList = []

# Creates VideoCapture thread
videoGetter = VideoGet()
videoGetter.start()

def trainerFunc():    
    for i in range(numEpisodes):
        stepNumber = 0

        print("Beginning episode",i,"...")
        currentState = env.reset(videoGetter) # Defines initial state of the system
        rAll = 0 # Quantifies rewards over time
        completeStatus = False # Defines if episode succeeded or failed

        # The Q-Table learning algorithm
        while stepNumber < 99: # Max amount of steps allowed before episode times out and fails
            
            
            if (cv.waitKey(1) == ord('q') or videoGetter.stopped):
                videoGetter.stop()

            print("Starting trial",stepNumber,"in state",currentState,"...")
            stepNumber += 1
            #Choose an action by greedily (with noise) picking from Q table

            # random noise added to each column element in row "s"
            # then the INDEX of the maximum of those is selected
            # np.argmax(Q[s,:] -- Searches table for action that is most likely to succeed given the state
            # np.random.randn(1,env.numActions)*(1./(i+1))) -- creates random noise in action selection that becomes less random as episodes continue
            # Selects an action to take
            action = np.argmax(Q[currentState,:] + np.random.randn(1,env.numActions)*(1./(i+1)))
            
            # Displays selected action
            print("Performing Action:",action)

            # Get new state and reward from environment
            # Takes new step using the action 'a', gets new state, reward (1 or 0), and whether or not the episode succeeded (True/False)
            # newState,reward,completeStatus = asyncio.run(asyncio.gather(env.step(action)))
            newState, reward, completeStatus = env.step(action, videoGetter)
            print(f"In Train, after Step(): New State: {newState}, Reward: {reward}, Complete Status {completeStatus}")
            if (reward==1):
                # SUCCESS. update the table and start again.
                print(f"New State: {newState}. Reward: {reward}, Goal Achieved!!\n")
            else:
                print(f"New State: {newState}. Reward: {reward}\n")

            

            #Update Q-Table with new knowledge
            Q[currentState,action] = Q[currentState,action] + learningRate*(reward + gamma*np.max(Q[newState,:]) - Q[currentState,action])
            rAll += reward
            currentState = newState

            # if success or fail, start a new episode
            if completeStatus == True:
                break

        rList.append(rAll)

    # Rotates the Q table 90 degrees, rounds values to fourth decimal, and prints table
    print(np.round(np.rot90(Q), decimals=4))
    videoGetter.stop()

def main():
    trainerFunc()
    
main()

