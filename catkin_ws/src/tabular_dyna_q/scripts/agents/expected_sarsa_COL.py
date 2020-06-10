#!/usr/bin/env python
# coding=utf-8
'''
@Author       : LI Jinjie
@Date         : 2020-05-18 17:52:10
@LastEditors  : LI Jinjie
@LastEditTime : 2020-05-20 18:01:43
@Units        : None
@Description  : This agent has TGT and COL two agent
@Dependencies : None
@NOTICE       : None
'''

import numpy as np
from template_agent import BaseAgent


# Expected Sarsa agent here


class ExpectedSarsaAgentCOL(BaseAgent):
    def agent_init(self, agent_init_info):
        """Setup for the agent called when the experiment first starts.

        Args:
        agent_init_info (dict), the parameters used to initialize the agent. The dictionary contains:
        {
            num_states (int): The number of states,
            num_actions (int): The number of actions,
            epsilon (float): The epsilon parameter for exploration,
            step_size (float): The step-size,
            discount (float): The discount factor,
        }

        """
        # Store the parameters provided in agent_init_info.
        self.num_actions = agent_init_info["num_actions"]
        self.num_states = agent_init_info["num_states"]
        self.epsilon = agent_init_info["epsilon"]
        self.step_size = agent_init_info["step_size"]
        self.discount = agent_init_info["discount"]
        q_path_TGT = agent_init_info.get(
            'q_path', "/home/ljj/gazebo_drone_tutorials/catkin_ws/src/tabular_dyna_q/scripts/results/DynaQ_table_r1_e150.npy")
        self.rand_generator = np.random.RandomState(agent_init_info["seed"])

        # Create an array for action-value estimates and initialize it to zero.
        # The array of action-value estimates.
        self.q_COL = np.zeros((self.num_states, self.num_actions))
        self.q_TGT = np.load(q_path_TGT)

        self.state_TGT = None

    def agent_start(self, state):
        """The first method called when the episode starts, called after
        the environment starts.
        Args:
            state (int): the state from the
                environment's evn_start function.
        Returns:
            action (int): the first action the agent takes.
        """
        state_TGT = self.state_TGT
        state_COL = state

        # Choose action using epsilon greedy.
        current_q_TGT = self.q_TGT[state_TGT, :]
        current_q_COL = self.q_COL[state_COL, :]
        # print 'current_q_TGT=', current_q_TGT
        # print 'current_q_COL=', current_q_COL
        # print 'q_sum=', current_q_TGT + current_q_COL

        if self.rand_generator.rand() < self.epsilon:
            action = self.rand_generator.randint(self.num_actions)
        else:
            # get action from both q_tables
            action = self.argmax(current_q_TGT + * current_q_COL)
        self.prev_state = state_COL
        self.prev_action = action
        return action

    def agent_step(self, reward, state):
        """A step taken by the agent.
        Args:
            reward (float): the reward received for taking the last action taken
            state (int): the state from the
                environment's step based on where the agent ended up after the
                last step.
        Returns:
            action (int): the action the agent is taking.
        """
        state_TGT = self.state_TGT
        state_COL = state

        print 'reward_COL', reward

        # Choose action using epsilon greedy.
        current_q_TGT = self.q_TGT[state_TGT, :]
        current_q_COL = self.q_COL[state_COL, :]

        # print 'current_q_COL:', current_q_COL

        if self.rand_generator.rand() < self.epsilon:
            action = self.rand_generator.randint(self.num_actions)
        else:
            action = self.argmax(current_q_TGT + 1 * current_q_COL)

        # Perform an update (~5 lines)
        ### START CODE HERE ###
        current_q = current_q_COL
        pi = np.ones(self.num_actions) * (self.epsilon / self.num_actions)
        num_q_max = sum(current_q == max(current_q))
        pi[current_q == max(current_q)] = (1 - self.epsilon) / \
            num_q_max + (self.epsilon / self.num_actions)

        value = 0
        for a in range(self.num_actions):
            value += pi[a] * self.q_COL[state_COL, a]

        self.q_COL[self.prev_state, self.prev_action] += self.step_size * \
            (reward + self.discount * value -
             self.q_COL[self.prev_state, self.prev_action])

        ### END CODE HERE ###

        self.prev_state = state_COL
        self.prev_action = action
        return action

    def agent_end(self, reward):
        """Run when the agent terminates.
        Args:
            reward (float): the reward the agent received for entering the
                terminal state.
        """
        print 'reward_COL', reward
        # Perform the last update in the episode (1 line)
        ### START CODE HERE ###
        self.q_COL[self.prev_state, self.prev_action] += self.step_size * \
            (reward - self.q_COL[self.prev_state, self.prev_action])
        ### END CODE HERE ###

    def argmax(self, q_values):
        """argmax with random tie-breaking
        Args:
            q_values (Numpy array): the array of action-values
        Returns:
            action (int): an action with the highest value
        """
        top = float("-inf")
        ties = []

        for i in range(len(q_values)):
            if q_values[i] > top:
                top = q_values[i]
                ties = []

            if q_values[i] == top:
                ties.append(i)

        return self.rand_generator.choice(ties)

    def agent_cleanup(self):
        """Cleanup done after the agent ends."""
        pass

    def agent_message(self, message):
        """A function used to pass information from the agent to the experiment.
        Args:
            message: The message passed to the agent.
        Returns:
            The response (or answer) to the message.
        """
        pass


if __name__ == "__main__":
    pass
