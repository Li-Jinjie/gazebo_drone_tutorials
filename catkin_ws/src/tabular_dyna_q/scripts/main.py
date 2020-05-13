#!/usr/bin/env python
# coding=utf-8
'''
@Author       : LI Jinjie
@Date         : 2020-05-07 10:18:06
@LastEditors  : LI Jinjie
@LastEditTime : 2020-05-09 22:47:22
@Units        : None
@Description  : file content
@Dependencies : None
@NOTICE       : None
'''
import shutil
import os
import sys
import time
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
    try:
        os.mkdir('results')
    except:
        pass
    # os.makedirs('results', exist_ok=True)

    agents = {
        "Q-learning": QLearningAgent,
        "Expected Sarsa": ExpectedSarsaAgent
    }
    env = GazeboEnvironment
    all_reward_sums = {}  # Contains sum of rewards during episode
    all_state_visits = {}  # Contains state visit counts during the last 10 episodes
    agent_info = {"num_actions": 10, "num_states": 121,
                  "epsilon": 0.1, "step_size": 0.5, "discount": 0.9}
    env_info = {"end_radius": 0.05, "target_x": 1.2, "target_y": -1.2}
    num_runs = 5  # The number of runs 原来是100
    # num_runs = 100  # The number of runs
    num_episodes = 100  # The number of episodes in each run 原来是500

    for algorithm in ["Expected Sarsa", "Q-learning"]:
        all_reward_sums[algorithm] = []
        all_state_visits[algorithm] = []
        for run in tqdm(range(num_runs)):
            agent_info["seed"] = run
            rl_glue = RLGlue(env, agents[algorithm])
            rl_glue.rl_init(agent_info, env_info)

            reward_sums = []
            state_visits = np.zeros(agent_info["num_states"])
    #         last_episode_total_reward = 0
            for episode in range(num_episodes):
                start_time = time.clock()
                if episode < num_episodes - 10:
                    # Runs an episode
                    rl_glue.rl_episode(0)
                else:
                    # Runs an episode while keeping track of visited states
                    state, action = rl_glue.rl_start()
                    state_visits[state] += 1
                    is_terminal = False
                    while not is_terminal:
                        # # stop the program
                        # line = sys.stdin.readline()
                        # print 'line=', line
                        # if line == 'q':
                        #     sys.exit()
                        reward, state, action, is_terminal = rl_glue.rl_step()
                        state_visits[state] += 1

                reward_sums.append(rl_glue.rl_return())
    #             last_episode_total_reward = rl_glue.rl_return()
                end_time = time.clock()
                print "The time of ", episode, " episode:", end_time - start_time

            print 'q_table:', rl_glue.agent.q
            all_reward_sums[algorithm].append(reward_sums)
            all_state_visits[algorithm].append(state_visits)

        name = 'results/' + algorithm + '_q_table_r5_e100.npy'
        np.save(name, rl_glue.agent.q)

    # save results
    np.save('results/q_learning_r5_e100.npy', all_reward_sums['Q-learning'])
    np.save('results/expected_sarsa_r5_e100.npy',
            all_reward_sums['Expected Sarsa'])

    np.save('results/all_state_visits_r5_e100.npy', all_state_visits)
