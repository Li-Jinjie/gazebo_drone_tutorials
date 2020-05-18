#!/usr/bin/env python
# coding=utf-8
'''
@Author       : LI Jinjie
@Date         : 2020-05-07 10:18:06
@LastEditors  : LI Jinjie
@LastEditTime : 2020-05-18 16:19:01
@Units        : None
@Description  : This file could train colision avoidance task on the basis of target seek ability.
@Dependencies : None
@NOTICE       : None
'''
import shutil
import os
import sys
import time
import numpy as np
from agents.dyna_q import DynaQAgent
from environments.gazebo_2_tasks_env import GazeboEnvironment2
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

    agent = DynaQAgent
    env = GazeboEnvironment2
    all_reward_sums = {}  # Contains sum of rewards during episode
    all_state_visits = {}  # Contains state visit counts during the last 10 episodes
    agent_info = {"num_actions": 10, "num_states": 121,
                  "epsilon": 0.1, "step_size": 0.5, "discount": 0.9, "planning_steps": 5}
    # env_info = {"end_radius": 0.05, "target_x": 1.2, "target_y": -1.2}
    env_info = {"end_radius": 0.05, 'robot_name': 'uav1',
                'random_flag': True, 'target_x': 1.2, 'target_y': -1.2, 'obstacle_x': 0.6, 'obstacle_y': 0.6}

    num_runs = 1  # The number of runs 原来是100
    num_episodes = 150  # The number of episodes in each run

    all_reward_sums = []
    all_episode_time = []
    all_state_visits = []
    for run in tqdm(range(num_runs)):

        agent_info["seed"] = run
        rl_glue = RLGlue(env, agent)
        rl_glue.rl_init(agent_info, env_info)
        path = "results/DynaQ_table_r2_e50.npy"
        rl_glue.agent.q_values = np.load(path)

        reward_sums = []
        episode_time = []
        state_visits = np.zeros(agent_info["num_states"])
        for episode in range(num_episodes):
            start_time = time.clock()

            # Runs an episode while keeping track of visited states
            state, action = rl_glue.rl_start()
            state_visits[state] += 1
            is_terminal = False
            while not is_terminal:
                reward, state, action, is_terminal = rl_glue.rl_step()
                state_visits[state] += 1

            end_time = time.clock()
            reward_sums.append(rl_glue.rl_return())
            episode_time.append(end_time - start_time)

            print "The time of ", episode, " episode:", end_time - start_time

        print 'q_table:', rl_glue.agent.q_values
        all_reward_sums.append(reward_sums)
        all_state_visits.append(state_visits)
        all_episode_time.append(episode_time)

    # save results

    np.save('results/DynaQ_table_r1_e150.npy', rl_glue.agent.q_values)
    np.save('results/DynaQ_r1_e150.npy', all_reward_sums)
    np.save('results/DynaQ_state_r1_e150.npy', all_state_visits)
    np.save('results/DynaQ_episode_time_r1_e150.npy', all_episode_time)
