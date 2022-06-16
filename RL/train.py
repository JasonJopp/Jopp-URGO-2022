import visual_servoing_env
import numpy as np

env = visual_servoing_env.ServoingEnvironment()            

#Initialize table with all zeros
Q = np.zeros([env.num_of_states,env.num_of_actions])

# Set learning parameters
lr = .8   
y = .95 # Gamma setting for updating the Q-table  
num_episodes = 2

#create lists to contain total rewards and steps per episode
#jList = []
rList = []

for i in range(num_episodes):
    print("Beginning episode",i,"...")
    #print(Q)

    #Reset environment and get initial observation
    s = env.reset() # Defines state of system
    rAll = 0 # Quantifying rewards over time
    d = False # Defines if episode succeeded or failed
    j = 0 # Step number

    #The Q-Table learning algorithm
    while j < 99: # Max amount of steps allowed before episode times out and fails
        print("Starting trial",j,"in state",s,"...")
        j+=1
        #Choose an action by greedily (with noise) picking from Q table

        # random noise added to each column element in row "s"
        # then the INDEX of the maximum of those is selected
        # np.argmax(Q[s,:] -- Searches table for action that is most likely to succeed given the state
        # np.random.randn(1,env.num_of_actions)*(1./(i+1))) -- creates random noise in action selection that becomes less random as episodes continue
        # Selects an action to take
        a = np.argmax(Q[s,:] + np.random.randn(1,env.num_of_actions)*(1./(i+1)))
        
        # Displays selected action
        print("Selected action",a)

        # Get new state and reward from environment
        s1,r,d = env.step(a) # Takes new step using the action 'a', gets new state, reward (1 or 0), and whether or not the episode succeeded (True/False)
        print("Action performed. In state",s1,". reward:",r)

        if (r==1):
            # SUCCESS. update the table and start again.
            print("Goal Achieved!!")

        print("NEW STATE",s1)

        #Update Q-Table with new knowledge
        Q[s,a] = Q[s,a] + lr*(r + y*np.max(Q[s1,:]) - Q[s,a])
        #print(Q)
        rAll += r
        #print("rAll:",rAll)
        s = s1

        # if success or fail, start a new episode
        if d == True: 
            break

    rList.append(rAll)
print("Training complete")
print(Q)
