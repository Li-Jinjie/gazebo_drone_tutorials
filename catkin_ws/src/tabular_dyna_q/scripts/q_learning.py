#!/usr/bin/env python
# coding=utf-8
'''
@Author       : LI Jinjie
@Date         : 2020-05-03 17:42:04
@LastEditors  : LI Jinjie
@LastEditTime : 2020-05-03 22:57:37
@Units        : None
@Description  : The implementation code of q-learning algorithm
@Dependencies : None
@NOTICE       : None
'''
import numpy as np
from template_agent import BaseAgent


# Q-Learning agent here


class QLearningAgent(BaseAgent):
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
        self.rand_generator = np.random.RandomState(agent_info["seed"])

        # Create an array for action-value estimates and initialize it to zero.
        # The array of action-value estimates.
        self.q = np.zeros((self.num_states, self.num_actions))

    def agent_start(self, state):
        """The first method called when the episode starts, called after
        the environment starts.
        Args:
            state (int): the state from the
                environment's evn_start function.
        Returns:
            action (int): the first action the agent takes.
        """

        # Choose action using epsilon greedy.
        current_q = self.q[state, :]
        if self.rand_generator.rand() < self.epsilon:
            action = self.rand_generator.randint(self.num_actions)
        else:
            action = self.argmax(current_q)
        self.prev_state = state
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

        # Choose action using epsilon greedy.
        current_q = self.q[state, :]
        if self.rand_generator.rand() < self.epsilon:
            action = self.rand_generator.randint(self.num_actions)
        else:
            action = self.argmax(current_q)

        # Perform an update (1 line)
        ### START CODE HERE ###
        self.q[self.prev_state, self.prev_action] += self.step_size * \
            (reward + self.discount *
             max(self.q[state, :]) - self.q[self.prev_state, self.prev_action])
        ### END CODE HERE ###

        self.prev_state = state
        self.prev_action = action
        return action

    def agent_end(self, reward):
        """Run when the agent terminates.
        Args:
            reward (float): the reward the agent received for entering the
                terminal state.
        """
        # Perform the last update in the episode (1 line)
        ### START CODE HERE ###
        self.q[self.prev_state, self.prev_action] += self.step_size * \
            (reward - self.q[self.prev_state, self.prev_action])
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


if __name__ == "__main__":
    # agents = {
    #     "Q-learning": QLearningAgent,
    #     "Expected Sarsa": ExpectedSarsaAgent
    # }
    # env = cliffworld_env.Environment
    all_reward_sums = {}  # Contains sum of rewards during episode
    all_state_visits = {}  # Contains state visit counts during the last 10 episodes
    agent_info = {"num_actions": 4*7, "num_states": 11*11,
                  "epsilon": 0.1, "step_size": 0.5, "discount": 0.9}
    env_info = {}
    num_runs = 100  # The number of runs
    num_episodes = 500  # The number of episodes in each run

    for algorithm in ["Q-learning", "Expected Sarsa"]:
        all_reward_sums[algorithm] = []
        all_state_visits[algorithm] = []
        for run in tqdm(range(num_runs)):
            agent_info["seed"] = run
            rl_glue = RLGlue(env, agents[algorithm])
            rl_glue.rl_init(agent_info, env_info)

            reward_sums = []
            state_visits = np.zeros(48)
    #         last_episode_total_reward = 0
            for episode in range(num_episodes):
                if episode < num_episodes - 10:
                    # Runs an episode
                    rl_glue.rl_episode(0)
                else:
                    # Runs an episode while keeping track of visited states
                    state, action = rl_glue.rl_start()
                    state_visits[state] += 1
                    is_terminal = False
                    while not is_terminal:
                        reward, state, action, is_terminal = rl_glue.rl_step()
                        state_visits[state] += 1

                reward_sums.append(rl_glue.rl_return())
    #             last_episode_total_reward = rl_glue.rl_return()

            all_reward_sums[algorithm].append(reward_sums)
            all_state_visits[algorithm].append(state_visits)
