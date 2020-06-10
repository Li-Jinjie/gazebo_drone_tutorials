#!/usr/bin/env python
# coding=utf-8
'''
@Author       : LI Jinjie
@Date         : 2020-05-07 10:18:06
@LastEditors  : LI Jinjie
@LastEditTime : 2020-05-19 23:51:15
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
from agents.expected_sarsa_COL import ExpectedSarsaAgentCOL
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

    agent = ExpectedSarsaAgentCOL
    env = GazeboEnvironment2
    all_reward_sums = {}  # Contains sum of rewards during episode
    all_state_visits = {}  # Contains state visit counts during the last 10 episodes
    agent_info = {"num_actions": 10, "num_states": 45,
                  "epsilon": 0.2, "step_size": 0.5, "discount": 0.9, "planning_steps": 5}
    # env_info = {"end_radius": 0.05, "target_x": 1.2, "target_y": -1.2}
    env_info = {"end_radius": 0.20, 'robot_name': 'uav1',
                'random_flag': False, 'target_x': 2.0, 'target_y': 2.0, 'obstacle_x': 1.0, 'obstacle_y': 1.0}
    # 训练了一组： 'target_x': 2.0, 'target_y': 2.0, 'obstacle_x': 0.8, 'obstacle_y': 0.6

    num_runs = 1  # The number of runs 原来是100
    num_episodes = 50  # The number of episodes in each run

    all_reward_sums = []
    all_episode_time = []
    all_state_visits = []
    for run in tqdm(range(num_runs)):

        agent_info["seed"] = run
        rl_glue = RLGlue(env, agent)
        rl_glue.rl_init(agent_info, env_info)
        path = "results/exSarsaCOL_table_tmp.npy"
        rl_glue.agent.q_COL = np.load(path)
        print 'q_table_COL', rl_glue.agent.q_COL
        # rl_glue.agent.q_values = np.load(path)

        reward_sums = []
        episode_time = []
        state_visits = np.zeros(agent_info["num_states"])
        for episode in range(num_episodes):
            start_time = time.clock()

            # Runs an episode while keeping track of visited states

            # 手动更新一下TGT任务的RL state
            env = rl_glue.environment
            rl_glue.agent.state_TGT = env.get_observation_TGT(
                env.current_state, env.target_position)

            state, action = rl_glue.rl_start()
            state_visits[state] += 1
            is_terminal = False
            while not is_terminal:
                rl_glue.agent.q_COL[36:45, :] = 0

                rl_glue.agent.state_TGT = env.get_observation_TGT(
                    env.current_state, env.target_position)

                reward, state, action, is_terminal = rl_glue.rl_step()
                state_visits[state] += 1

            np.save('results/exSarsaCOL_table_tmp.npy',   # exSarsaCOL_table_tmp.npy   exSarsaCOL_table_tmp.npy
                    rl_glue.agent.q_COL)

            end_time = time.clock()
            reward_sums.append(rl_glue.rl_return())
            episode_time.append(end_time - start_time)

            print "The time of ", episode, " episode:", end_time - start_time

        # print 'q_table_COL:', rl_glue.agent.q_COL
        all_reward_sums.append(reward_sums)
        all_state_visits.append(state_visits)
        all_episode_time.append(episode_time)

    # save results

    np.save('results/exSarsaCOL_table_r1_e50_4.npy', rl_glue.agent.q_COL)
    np.save('results/exSarsaCOL_r1_e50_4.npy', all_reward_sums)
    np.save('results/exSarsaCOL_state_r1_e50_4.npy', all_state_visits)
    np.save('results/exSarsaCOL_episode_time_r1_e50_4.npy', all_episode_time)
