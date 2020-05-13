#!/usr/bin/env python
# coding=utf-8
'''
@Author       : LI Jinjie
@Date         : 2020-05-07 10:18:06
@LastEditors  : LI Jinjie
@LastEditTime : 2020-05-10 09:50:17
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
from agents.dyna_q_plus import DynaQPlusAgent
from agents.dyna_q import DynaQAgent
from environments.gazebo_env import GazeboEnvironment
from scipy.stats import sem
import matplotlib.pyplot as plt
from rl_glue import RLGlue
from tqdm import tqdm


def run_experiment(env, agent, env_parameters, agent_parameters, exp_parameters):

    # Experiment settings
    num_runs = exp_parameters['num_runs']
    num_episodes = exp_parameters['num_episodes']
    planning_steps_all = agent_parameters['planning_steps']

    env_info = env_parameters
    agent_info = {"num_states": agent_parameters["num_states"],  # We pass the agent the information it needs.
                  "num_actions": agent_parameters["num_actions"],
                  "epsilon": agent_parameters["epsilon"],
                  "discount": env_parameters["discount"],
                  "step_size": agent_parameters["step_size"]}

    # for collecting metrics
    all_averages = np.zeros((len(planning_steps_all), num_runs, num_episodes))
    # that shall be plotted later
    log_data = {'planning_steps_all': planning_steps_all}

    for idx, planning_steps in enumerate(planning_steps_all):

        print('Planning steps : ', planning_steps)
        # to prevent tqdm printing out-of-order before the above print()
        os.system('sleep 0.5')
        agent_info["planning_steps"] = planning_steps

        for i in tqdm(range(num_runs)):

            agent_info['random_seed'] = i
            agent_info['planning_random_seed'] = i

            # Creates a new RLGlue experiment with the env and agent we chose above
            rl_glue = RLGlue(env, agent)
            # We pass RLGlue what it needs to initialize the agent and environment
            rl_glue.rl_init(agent_info, env_info)

            for j in range(num_episodes):

                # We start an episode. Here we aren't using rl_glue.rl_episode()
                rl_glue.rl_start()
                # like the other assessments because we'll be requiring some
                # data from within the episodes in some of the experiments here
                is_terminal = False
                num_steps = 0
                while not is_terminal:
                    # The environment and agent take a step
                    reward, _, action, is_terminal = rl_glue.rl_step()
                    # and return the reward and action taken.
                    num_steps += 1

                all_averages[idx][i][j] = num_steps

    log_data['all_averages'] = all_averages
    np.save("results/Dyna-Q_planning_steps.npy", log_data)
    np.save("results/Dyna_Q_table.npy", rl_glue.agent.q_values)


def plot_steps_per_episode(file_path):

    data = np.load(file_path).item()
    all_averages = data['all_averages']
    planning_steps_all = data['planning_steps_all']

    for i, planning_steps in enumerate(planning_steps_all):
        plt.plot(np.mean(all_averages[i], axis=0),
                 label='Planning steps = '+str(planning_steps))

    plt.legend(loc='upper right')
    plt.xlabel('Episodes')
    plt.ylabel('Steps\nper\nepisode', rotation=0, labelpad=40)
    plt.axhline(y=16, linestyle='--', color='grey', alpha=0.4)
    plt.show()


def run_experiment_with_state_visitations(env, agent, env_parameters, agent_parameters, exp_parameters, result_file_name):

    # Experiment settings
    num_runs = exp_parameters['num_runs']
    num_max_steps = exp_parameters['num_max_steps']
    planning_steps_all = agent_parameters['planning_steps']

    env_info = {"change_at_n": env_parameters["change_at_n"]}
    agent_info = {"num_states": agent_parameters["num_states"],
                  "num_actions": agent_parameters["num_actions"],
                  "epsilon": agent_parameters["epsilon"],
                  "discount": env_parameters["discount"],
                  "step_size": agent_parameters["step_size"]}

    state_visits_before_change = np.zeros(
        (len(planning_steps_all), num_runs, 54))  # For saving the number of
    state_visits_after_change = np.zeros(
        (len(planning_steps_all), num_runs, 54))  # state-visitations
    # For saving the cumulative reward
    cum_reward_all = np.zeros(
        (len(planning_steps_all), num_runs, num_max_steps))
    log_data = {'planning_steps_all': planning_steps_all}

    for idx, planning_steps in enumerate(planning_steps_all):

        print('Planning steps : ', planning_steps)
        # to prevent tqdm printing out-of-order before the above print()
        os.system('sleep 1')
        # We pass the agent the information it needs.
        agent_info["planning_steps"] = planning_steps

        for run in tqdm(range(num_runs)):

            agent_info['random_seed'] = run
            agent_info['planning_random_seed'] = run

            # Creates a new RLGlue experiment with the env and agent we chose above
            rl_glue = RLGlue(env, agent)
            # We pass RLGlue what it needs to initialize the agent and environment
            rl_glue.rl_init(agent_info, env_info)

            num_steps = 0
            cum_reward = 0

            while num_steps < num_max_steps-1:

                state, _ = rl_glue.rl_start()  # We start the experiment. We'll be collecting the
                # state-visitation counts to visiualize the learned policy
                is_terminal = False
                if num_steps < env_parameters["change_at_n"]:
                    state_visits_before_change[idx][run][state] += 1
                else:
                    state_visits_after_change[idx][run][state] += 1

                while not is_terminal and num_steps < num_max_steps-1:
                    reward, state, action, is_terminal = rl_glue.rl_step()
                    num_steps += 1
                    cum_reward += reward
                    cum_reward_all[idx][run][num_steps] = cum_reward
                    if num_steps < env_parameters["change_at_n"]:
                        state_visits_before_change[idx][run][state] += 1
                    else:
                        state_visits_after_change[idx][run][state] += 1

    log_data['state_visits_before'] = state_visits_before_change
    log_data['state_visits_after'] = state_visits_after_change
    log_data['cum_reward_all'] = cum_reward_all
    np.save("results/" + result_file_name, log_data)


if __name__ == "__main__":
    # Experiment parameters
    experiment_parameters = {
        "num_runs": 5,                     # The number of times we run the experiment
        "num_episodes": 40,                 # The number of episodes per experiment
    }

    # Environment parameters
    environment_parameters = {
        "discount": 0.9,
    }

    # Agent parameters
    agent_parameters = {
        "num_states": 121,
        "num_actions": 10,
        "epsilon": 0.1,
        "step_size": 0.5,
        # The list of planning_steps we want to try
        "planning_steps": [0, 5, 50]
    }

    current_env = GazeboEnvironment   # The environment
    current_agent = DynaQAgent              # The agent

    run_experiment(current_env, current_agent, environment_parameters,
                   agent_parameters, experiment_parameters)
    plot_steps_per_episode('results/Dyna-Q_planning_steps.npy')

    # run_experiment_with_state_visitations(
    #     current_env, current_agent, environment_parameters, agent_parameters, experiment_parameters, "Dyna-Q_shortcut_steps")
    # plot_cumulative_reward('results/Dyna-Q_shortcut_steps.npy', 'planning_steps_all', 'cum_reward_all',
    #    'Cumulative\nreward', 'Planning steps = ', 'Dyna-Q : Varying planning_steps')
