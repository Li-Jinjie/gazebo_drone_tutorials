#!/usr/bin/env python
# coding=utf-8
'''
@Author       : LI Jinjie
@Date         : 2020-05-07 10:18:06
@LastEditors  : LI Jinjie
@LastEditTime : 2020-05-07 15:43:49
@Units        : None
@Description  : file content
@Dependencies : None
@NOTICE       : None
'''
import shutil
import os
import sys
import numpy as np
from q_learning import QLearningAgent
from expected_sarsa import ExpectedSarsaAgent
from gazebo_env import GazeboEnvironment
from scipy.stats import sem
import matplotlib.pyplot as plt
from rl_glue import RLGlue
from tqdm import tqdm


if __name__ == "__main__":
    # Do not modify this cell!

    agents = {
        "Q-learning": QLearningAgent,
        "Expected Sarsa": ExpectedSarsaAgent
    }
    env = GazeboEnvironment
    all_reward_sums = {}  # Contains sum of rewards during episode
    all_state_visits = {}  # Contains state visit counts during the last 10 episodes
    agent_info = {"num_actions": 10, "num_states": 121,
                  "epsilon": 0.1, "step_size": 0.5, "discount": 0.9}
    env_info = {"end_radius": 0.05, "target_x": 1.0, "target_y": 1.0}
    num_runs = 10  # The number of runs
    # num_runs = 100  # The number of runs
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
                        # stop the program
                        line = sys.stdin.readline()
                        print 'line=', line
                        if line == 'q':
                            sys.exit()
                        reward, state, action, is_terminal = rl_glue.rl_step()
                        state_visits[state] += 1

                reward_sums.append(rl_glue.rl_return())
    #             last_episode_total_reward = rl_glue.rl_return()

            all_reward_sums[algorithm].append(reward_sums)
            all_state_visits[algorithm].append(state_visits)

    # save results
    os.makedirs('results', exist_ok=True)
    np.save('results/q_learning.npy', all_reward_sums['Q-learning'])
    np.save('results/expected_sarsa.npy', all_reward_sums['Expected Sarsa'])

    for algorithm in ["Q-learning", "Expected Sarsa"]:
        plt.plot(np.mean(all_reward_sums[algorithm], axis=0), label=algorithm)
    plt.xlabel("Episodes")
    plt.ylabel("Sum of\n rewards\n during\n episode", rotation=0, labelpad=40)
    plt.xlim(0, 500)
    plt.ylim(-100, 0)
    plt.legend()
    plt.show()

    for algorithm, position in [("Q-learning", 211), ("Expected Sarsa", 212)]:
        plt.subplot(position)
        average_state_visits = np.array(
            all_state_visits[algorithm]).mean(axis=0)
        grid_state_visits = average_state_visits.reshape((4, 12))
        grid_state_visits[0, 1:-1] = np.nan
        plt.pcolormesh(grid_state_visits, edgecolors='gray', linewidth=2)
        plt.title(algorithm)
        plt.axis('off')
        cm = plt.get_cmap()
        cm.set_bad('gray')

        plt.subplots_adjust(bottom=0.0, right=0.7, top=1.0)
        cax = plt.axes([0.85, 0.0, 0.075, 1.])
    cbar = plt.colorbar(cax=cax)
    cbar.ax.set_ylabel("Visits during\n the last 10\n episodes",
                       rotation=0, labelpad=70)
    plt.show()
