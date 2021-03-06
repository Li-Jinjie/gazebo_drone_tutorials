#!/usr/bin/env python
# coding=utf-8
'''
@Author       : LI Jinjie
@Date         : 2020-05-03 17:42:04
@LastEditors  : LI Jinjie
@LastEditTime : 2020-05-20 19:45:54
@Units        : None
@Description  : The agent whose actions depend totally on the q table.
@Dependencies : None
@NOTICE       : None
'''
import numpy as np
from template_agent import BaseAgent


# Q-Learning agent here


class Agent_COL_TGT(BaseAgent):
    """Implements the agent for an RLGlue environment

    Note:
        env_init, env_start, env_step, env_cleanup, and env_message are required
        methods.
    """

    def agent_init(self, agent_init_info):
        """Setup for the agent called when the experiment first starts.

        Args:
        agent_init_info (dict), the parameters used to initialize the agent. The dictionary contains:
        {
            q_path (string): The absolute path of a q-table file.     
        }

        """
        # Store the parameters provided in agent_init_info.

        q_TGT_path = agent_init_info.get(
            'q_path', "/home/ljj/gazebo_drone_tutorials/catkin_ws/src/tabular_dyna_q/scripts/results/DynaQ_table_r1_e150.npy")
        q_COL_path = agent_init_info.get(
            'q_path', "/home/ljj/gazebo_drone_tutorials/catkin_ws/src/tabular_dyna_q/scripts/results/exSarsaCOL_table_tmp.npy")

        # The array of action-value estimates.
        self.q_TGT = np.load(q_TGT_path)
        self.q_COL = np.load(q_COL_path)
        self.q_COL[36:45, :] = 0

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

        # Choose action using q-table.
        state_TGT = self.state_TGT
        state_COL = state

        current_q_TGT = self.q_TGT[state_TGT, :]
        current_q_COL = self.q_COL[state_COL, :]

        action = self.argmax(current_q_TGT + 1.5 * current_q_COL)

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

        current_q_TGT = self.q_TGT[state_TGT, :]
        current_q_COL = self.q_COL[state_COL, :]

        # print 'current_q_COL', current_q_COL

        action = self.argmax(current_q_TGT + 1.5 * current_q_COL)

        return action

    def agent_end(self, reward):
        """Run when the agent terminates.
        Args:
            reward (float): the reward the agent received for entering the
                terminal state.
        """
        pass

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

        # return self.rand_generator.choice(ties)
        return np.random.choice(ties)

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
