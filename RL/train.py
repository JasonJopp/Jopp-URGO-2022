import servoingEnvironment
import numpy as np
import sys
import cv2 as cv
from videoGet import VideoGet
from datetime import date

env = servoingEnvironment.ServoingEnvironment()            

#Initialize Q-table with all zeros
Q = np.zeros([env.numStates,env.numActions])

# Track how many times the system is in each state-action entry
Qcount = np.zeros([env.numStates,env.numActions])

# Set learning parameters
learningRate = .8
gamma = .95 # Gamma setting for updating the Q-table  
numEpisodes = 5

# Create lists to contain total rewards and steps per episode
rList = []

# Creates VideoCapture thread for getting frames from camera
videoGetter = VideoGet()
videoGetter.start()

def printQ(QU):
    for state in range(env.numStates):
        for action in range(env.numActions):
            print(f'{QU[state][action]:>5.2f}',end=' ')
        print()

def fill_Q(filename):
    """
    Fills Q Table using filename specified in cmdLine when running program.
    """
    try:
        f = open(filename,'r')
        lines = f.read().splitlines()
        f.close()
    
    except:
        print("ERROR opening and reading file")
        print("***** IGNORING. Starting with blank Q table.")

    row = 0

    for line in lines:
        entries = line.split()
        for column in range(env.numActions):
            #print(entries[row],end=' ')
            if entries[column] != '---':
                Q[row][column] = float(entries[column])
        #print(Q)
        row += 1
    print(Q)
    input('PAUSED, waiting for input..')

def writeQ(filename, Q, threshold, ints=False):
    filename = filename+'-'+str(date.today())+'.txt'
    try:
        f = open(filename,"w")
    except:
        print('Error opening file.')
        # print Q to screen so it is not lost
        return

    # Overwriting threshold so all values print
    threshold = -500000
    # add space of header row to align with columns
    f.write('       ')

    # Prints a header for each column
    for state in range(env.numStates):
        # use image division state as header. determine if it is time to print
        if state%len(env.distanceStateDict) == 0:
            f.write(f'{(state//len(env.distanceStateDict)):>5} ')
        else:
            # for all blob ratio states, draw a border for the table
            f.write('_____ ')
    
    f.write('\n')

    # print the Q table values in row-column form
    for action in range(env.numActions):
        f.write(f'{action:>5}: ')
        for state in range(env.numStates):
            if Q[state,action]==0 or Q[state,action]<threshold:
                f.write(' ---  ')
            else:
                if ints:
                    f.write(f'{Q[state,action]:>5.0f} ')
                else:
                    f.write(f'{Q[state,action]:5.2f} ')
        f.write('\n')

    #Prints the highest valued action for each state from the Q table.
    # add some space to align with columns in printed Q table
    f.write('    ')
    for state in range(env.numStates):
        f.write(f'{np.argmax(Q[state,:]):>5}')
    
    f.write('\n\n')

    # Writes a qTable that can be used in future tests
    for state in range(env.numStates):
        for action in range(env.numActions):
            f.write(f'{Q[state][action]:>5.2f} ')
        f.write("\n")

    f.close()


def trainerFunc(Qout_file):   
    for i in range(numEpisodes):
        stepNumber = 0

        print("Beginning episode",i,"...")
        # Defines initial state of the system
        currentState = env.reset(videoGetter)

        # Quantifies rewards over time
        rAll = 0

        # Defines if episode succeeded or failed
        completeStatus = False

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
            # Debugging print(f"In Train, after Step(): New State: {newState}, Reward: {reward}, Complete Status {completeStatus}")
            if (reward==1):
                # SUCCESS. update the table and start again.
                print(f"New State: {newState}. Reward: {reward}, Goal Achieved!!\n")
            else:
                print(f"New State: {newState}. Reward: {reward}\n")

            

            # Update Q-Table with new knowledge
            Q[currentState,action] = Q[currentState,action] + learningRate*(reward + gamma*np.max(Q[newState,:]) - Q[currentState,action])
            # Updates Qcount table
            Qcount[currentState,action] += 1
            rAll += reward
            currentState = newState

            # if success or fail, start a new episode
            if completeStatus == True:
                break

        rList.append(rAll)

    # Rotates the Q table 90 degrees, rounds values to fourth decimal, and prints table
    print(np.round(np.rot90(Q), decimals=4))
    videoGetter.stop()
    if Qout_file:
        # Writes the Q table and other data to selected outfile
        writeQ(Qout_file, Q, -10)

def main():
    # By default, Q table will not import/export without correct arguements
    Qin_file = None
    Qout_file = None

    # Checks if filenames were passed in for Q table upon running the program
    if len(sys.argv) > 1:
        
        # Uses cmdline args indexes to find input/output filenames for Q Table
        for idx in range(len(sys.argv)):
            if sys.argv[idx] == '-f':
                Qin_file = sys.argv[idx+1]

            if sys.argv[idx] == '-o':
                Qout_file = sys.argv[idx+1]
        
        # Displays what has been selected.   
        print(f'Using {Qin_file} for Q table.')
        print(f'Output Q table to {Qout_file}.')
    
    if Qin_file:
        fill_Q(Qin_file)
    trainerFunc(Qout_file)
    printQ(Q)
    
if __name__ == "__main__":
    main()
