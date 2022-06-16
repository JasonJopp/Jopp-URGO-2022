import servoingEnvironment
import numpy as np

env = servoingEnvironment.ServoingEnvironment()            

#Initialize Q-table with all zeros
Q = np.zeros([env.num_of_states,env.num_of_actions])

# Set learning parameters
learningRate = .8   
gamma = .95 # Gamma setting for updating the Q-table  
num_episodes = 2

#create lists to contain total rewards and steps per episode
#jList = []
rList = []

for i in range(num_episodes):
    print("Beginning episode",i,"...")
    #print(Q)

    #Reset environment and get initial observation
    currentState = env.reset() # Defines state of system
    rAll = 0 # Quantifies rewards over time
    winState = False # Defines if episode succeeded or failed
    stepNumber = 0

    #The Q-Table learning algorithm
    while stepNumber < 99: # Max amount of steps allowed before episode times out and fails
        print("Starting trial",stepNumber,"in state",currentState,"...")
        stepNumber+=1
        #Choose an action by greedily (with noise) picking from Q table

        # random noise added to each column element in row "s"
        # then the INDEX of the maximum of those is selected
        # np.argmax(Q[s,:] -- Searches table for action that is most likely to succeed given the state
        # np.random.randn(1,env.num_of_actions)*(1./(i+1))) -- creates random noise in action selection that becomes less random as episodes continue
        # Selects an action to take
        action = np.argmax(Q[currentState,:] + np.random.randn(1,env.num_of_actions)*(1./(i+1)))
        
        # Displays selected action
        print("Selected action",action)

        # Get new state and reward from environment
        newState,reward,winState = env.step(action) # Takes new step using the action 'a', gets new state, reward (1 or 0), and whether or not the episode succeeded (True/False)
        print("Action performed. In state",newState,". reward:",reward)

        if (reward==1):
            # SUCCESS. update the table and start again.
            print("Goal Achieved!!")

        print("NEW STATE",newState)

        #Update Q-Table with new knowledge
        Q[currentState,action] = Q[currentState,action] + learningRate*(reward + gamma*np.max(Q[newState,:]) - Q[currentState,action])
        #print(Q)
        rAll += reward
        #print("rAll:",rAll)
        currentState = newState

        # if success or fail, start a new episode
        if winState == True:
            break

    rList.append(rAll)

print("Training complete")
print(Q)
