"""
Reinforcement learning Framework code

1) Forward
Get State(=lidar vector) and return Action(=velocity vector)

2) Train (Back-propagation)
Get State, Action, Next State, Reward & train Actor&Critic networks
"""

import os
import random
import time

import numpy as np
import matplotlib
import matplotlib.pyplot as plt

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from collections import deque
from torch.distributions import Normal

import Environment #user-defined environment



# Hyperparameters
num_episodes = 10000
learning_rate = 0.0001
gamma = 0.99
TAU = 0.1
batch_size = 32
buffer_size = 100000
num_epochs = 2
state_dim = 360 #lidar data
action_dim = 2 # forward backward, left right
work_dir = "C:/kisang/Ant_control/result_single_contact"



#define some useful functions: time, plot, flatten
def get_today():
    now = time.localtime()
    s = "%02d_%02d-%02d_%02d" % (now.tm_mon, now.tm_mday, now.tm_hour, now.tm_min)
    return s

def plot(reward, dist, timestep, total_reward, flag):
    if timestep%10 == 0:
        plt.figure(2)
        plt.cla() #delete all
        #durations_t = torch.tensor(laptimes, dtype=torch.float) #torch float tensor for duration
        plt.title('Result plot')
        plt.xlabel('timestep / 10')
        plt.ylabel('Total Reward')
        plt.plot(reward) # torch tensor to numpy
        plt.plot(dist)
        plt.pause(0.01)

    if flag==1:
        save_path = os.path.join(work_dir + "/result_plot_" + str(round(total_reward,2)) + "_" + str(timestep) + ".png")
        plt.savefig(save_path)


def flat_vectorize(state, batch_size):
    flattened = []

    if batch_size == 1: #for actions.... forwarding
        temp = []
        for q in state:
            for item in q:
                temp.append(item)
        return torch.FloatTensor(temp)

    for i in range(batch_size):
        temp = []
        #qpos, qvel, force, distance = state[i]
        
        for q in state[i]:
            for item in q:
                temp.append(item)

        flattened.append(temp)
        #print("flattened, temp:", len(flattened), len(temp))

    flattened = torch.FloatTensor(flattened)
    return flattened




"""
Define two networks: Ator and Critic
Actor: get lidar vector, return velocity vector
Critic: get lidar vector + velocity vector, return Q value

Define replay buffer
Replay Buffer: save histories for learning stability
"""

class Actor(nn.Module):
    def __init__(self, state_dim, action_dim): #state_dim = lidar vector, action_dim = velocity vector
        super(Actor, self).__init__()
        self.fc1 = nn.Linear(state_dim, 256)
        self.fc2 = nn.Linear(256, 256)
        self.fc3 = nn.Linear(256, action_dim)

    def forward(self, x):
        x = F.tanh(self.fc1(x))
        x = F.tanh(self.fc2(x))
        x = F.tanh(self.fc3(x))
        return x/2

class Critic(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(Critic, self).__init__()
        self.fc1 = nn.Linear(state_dim + action_dim, 256)
        self.fc2 = nn.Linear(256, 256)
        self.fc3 = nn.Linear(256, 1) #return Q value

    def forward(self, state, action):
        x = torch.cat([state, action], dim=1)
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        return self.fc3(x)


class ReplayBuffer:
    def __init__(self, max_size):
        self.buffer = deque(maxlen=max_size)

    def add(self, state, action, next_state, reward, done):
        self.buffer.append((state, action, next_state, reward, done))

    def sample(self, batch_size):
        batch = random.sample(self.buffer, batch_size) 
        states, actions, next_states, rewards, dones = zip(*batch)

        return (
            states,
            torch.FloatTensor(actions),
            next_states,
            torch.FloatTensor(rewards).unsqueeze(1),
            torch.FloatTensor(dones).unsqueeze(1)
            )

    def size(self):
        return len(self.buffer)




"""
Agent that can Act & Train
In the project: Agent = Car
"""
class Agent:
    def __init__ (self, state_dim, action_dim):

        #critic part
        self.critic = Critic(state_dim, action_dim)
        self.critic_target = Critic(state_dim, action_dim) #target network -> RL method for stablized learning
        self.critic_target.load_state_dict(self.critic.state_dict())
        self.critic_optimizer = optim.Adam(self.critic.parameters(), lr=learning_rate)

        #actor part
        self.actor = Actor(state_dim, action_dim)
        self.actor_target = Actor(state_dim, action_dim)
        self.actor_target.load_state_dict(self.actor.state_dict())
        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=learning_rate)

        #buffer
        self.Qbuffer = ReplayBuffer(buffer_size)


    def action (self, in_state): #forward(=state to action)

        state = flat_vectorize(in_state, 1)
        out = self.actor(state).detach().numpy()
        
        return out
        

    def train (self):
        if self.Qbuffer.size() < batch_size:
            print("short")
            return

        """critic training"""
        states, actions, next_states, rewards, dones = self.Qbuffer.sample(batch_size) #sampling from Qbuffer

        #vectorizing states
        states = flat_vectorize(states, batch_size)
        next_states = flat_vectorize(next_states, batch_size)

        with torch.no_grad():
            next_actions = self.actor_target(next_states)
            target_q = rewards + (1 - dones) * gamma * self.critic_target(next_states, next_actions) #make target Q value to learn

        actions = torch.FloatTensor(actions)
        current_q = self.critic(states, actions) #make current Q value to compare -> make loss
        critic_loss = nn.MSELoss()(current_q, target_q)

        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        self.critic_optimizer.step()


        """actor training"""
        current_action = self.actor(states)
        actor_loss = -self.critic(states, current_action).mean()

        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        self.actor_optimizer.step()


        """target update"""
        for param, target_param in zip(self.critic.parameters(), self.critic_target.parameters()):
            target_param.data.copy_(TAU * param.data + (1 - TAU) * target_param.data)

        for param, target_param in zip(self.actor.parameters(), self.actor_target.parameters()):
            target_param.data.copy_(TAU * param.data + (1 - TAU) * target_param.data)


    def return_net(self, num): #returning network parameters
        
        today = get_today()

        torch.save(self.actor.state_dict(), work_dir + "/actor" + "_" + str(num) + "_" +str(today)+".pt")
        highest_speed = num

        print("******************************************************")
        print("success case returned, highest_speed:", highest_speed)
        print("******************************************************")





"""
main

make Agent, load environment from the other code
receive data from simulator & execute training
"""

def main():
    #cuda
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    #load environment
    env = Environment()

    #declare agent
    agent = Agent(state_dim, action_dim)
    

    """episode loop
    episode: one trial from initializing state to terminalizing state"""
    for episode in range(num_episodes):

        #pre-execution
        states, actions, rewards, next_states, dones = [], [], [], [], []
        total_reward = 0
        success = 0
        timestep =0
        
        #initialize environment
        env.reset()
        state, action, next_state, reward, done_mask, success = env.step(np.zeros(action_dim))
        action = np.array(action)


        """execute one episode"""
        while done_mask == 0:
            
            action = agent.action(state)
            state, action, next_state, reward, done_mask, success = env.step(action)
            action = np.array(action)
            agent.Qbuffer.add(state, action, next_state,reward, done_mask) #save informations to Qbuffer
            
            #take next step
            state = next_state
            total_reward += reward
            timestep+=1

            
            """train every 10 timestep"""
            if timestep%10 == 0:
                for i in range(num_epochs):
                    agent.train()


        #if success, save data with pt file
        if success == 1:
            num = env.return_self_action_num()
            agent.return_net(num)
        print("episode end:", episode)


    print ("all episodes executed")



if __name__ == '__main__':
    main()
