# This file is intended for manual analysis of experiments
import random
import sys, os
sys.path.append("..")

import json
import shutil
from tqdm import tqdm
import matplotlib.pyplot as plt

from scripts.load_ros_data import *
import scripts.visuals, scripts.metrics
from scripts.helpers import *
from scripts.compare import clean_simulation_name, clean_method_name

# @todo: Put somewhere


"""
Analysis functions (select through argument)
"""

def load_data(simulation, planner_name):
    filename = simulation + "_" + planner_name
    data_folder, metric_folder, figure_folder = get_folder_names()
    data_path = data_folder + filename + ".txt"

    metric_folder = metric_folder + simulation
    metric_path = metric_folder + "/" + planner_name + ".json"
    with open(metric_path) as f:
        metrics = dict(json.load(f))

    data = load_ros_data(data_path)
    settings = get_settings()
    experiment_data = split_ros_data(data, num_experiments=235, filter_faulty=settings.filter_faulty)
    return data, experiment_data, metrics, figure_folder

def boxplot_of_multiple_simulations_and_planners(simulations, planners, figure_base_name, reload_cache=False, xmin=11.5, xmax=25.):
    data_folder, metric_folder, figure_folder = get_folder_names()

    simulations *= len(planners)  # Repeat the simulations for each planner

    def duplicate(testList, n):
        return [ele for ele in testList for _ in range(n)]

    n_planners = len(planners)
    planners = duplicate(planners, int(len(simulations) / len(planners)))  # Duplicate planners repeating the same planner N times

    assert len(planners) == len(simulations), "after copying there should be equal number of entries in simulations and planners"

    data_names = ["Task Duration", "Infeasibility"]
    data_functions = [
        lambda experiment_data, metrics: [e for e in merge_experiment_data(experiment_data, "metric_duration") if (e > 10. and e < 39.)],
        lambda experiment_data, metrics: metrics['num infeasible']]

    # Load the data from a cache if it exists (this prevents the slow loading of all planner data)
    if not reload_cache:
        simulation_data, cache_exists = load_from_cache(simulations, planners)
    else:
        cache_exists = False

    if not cache_exists:
        simulation_data = merge_simulation_data(simulations, planners, data_names, data_functions)
        save_in_cache(simulations, planners, simulation_data)

    for sim in range(len(simulations)):
        simulations[sim] = clean_simulation_name(simulations[sim])

    for planner in range(len(planners)):
        planners[planner] = clean_method_name(planners[planner])
    # fig = scripts.visuals.comparitive_boxplot(simulation_data[data_names[0]], data_names[0], planners, "Planner", simulations)
    fig = scripts.visuals.comparative_boxplot_simulations_planners(simulation_data, data_names[0], simulations, planners)
    ax = fig.gca()
    ax.set_xlim([xmin, xmax])
    ax.set_xlabel("Task Duration (s)")
    scripts.visuals.save_figure_as(fig, figure_folder, figure_base_name + "_duration", ratio=n_planners*0.25/1.5)

    # fig = scripts.visuals.comparitive_boxplot(simulation_data[data_names[1]], data_names[1], planners, "Planner", simulations)
    fig = scripts.visuals.comparative_boxplot_simulations_planners(simulation_data, data_names[1], simulations, planners)
    scripts.visuals.save_figure_as(fig, figure_folder, figure_base_name + "_infeasibility", ratio=n_planners*0.25/2)

def debug_visual(simulation, planner_name):
    data, experiment_data, metrics, figure_folder = load_data(simulation, planner_name)
    import matplotlib.pyplot as plt

    assert planner_name == "GMPCC" or planner_name == "GMPCCNO", "Can only use GMPCC for this analysis"

    # Verify active constraints
    signal_list = []
    for p in range(int(metrics['num paths']['max'])+1):
        signal_list.append('active_constraints_' + str(p))

    # scripts.visuals.compare_signals(experiment_data, signal_list, highlight_signal_list=[])
    scripts.visuals.plot_multiple_signals(experiment_data, signal_list, "Iteration", "Active Constraints", use_subplots=True)
    plt.show()


def visualize_intrusion(simulations, planners, base_figure_name):
    ata_folder, metric_folder, figure_folder = get_folder_names()

    simulations *= len(planners)  # Repeat the simulations for each planner
    def duplicate(testList, n):
        return [ele for ele in testList for _ in range(n)]
    n_planners = len(planners)
    planners = duplicate(planners, int(len(simulations) / len(planners)))  # Duplicate planners repeating the same planner N times
    assert len(planners) == len(simulations), "after copying there should be equal number of entries in simulations and planners"

    data_names = ["Intrusion"]
    data_functions = [lambda experiment_data, metrics: merge_experiment_data(experiment_data, "max_intrusion")]

    # Load the data from a cache if it exists (this prevents the slow loading of all planner data)
    if not reload_cache:
        simulation_data, cache_exists = load_from_cache(simulations, planners)
    else:
        cache_exists = False

    if not cache_exists:
        simulation_data = merge_simulation_data(simulations, planners, data_names, data_functions)
        save_in_cache(simulations, planners, simulation_data)

    for sim in range(len(simulations)):
        simulations[sim] = clean_simulation_name(simulations[sim])

    for planner in range(len(planners)):
        planners[planner] = clean_method_name(planners[planner])
    # fig = scripts.visuals.comparitive_boxplot(simulation_data[data_names[0]], data_names[0], planners, "Planner", simulations)
    fig = scripts.visuals.comparative_boxplot_simulations_planners(simulation_data, data_names[0], simulations, planners)
    ax = fig.gca()
    ax.set_xlabel("Intrusion (m)")
    scripts.visuals.save_figure_as(fig, figure_folder, f"{base_figure_name}_intrusion", ratio=n_planners * 0.25 / 1.5)

def plot_trajectories(simulation, planner_name):
    data, experiment_data, metrics, figure_folder = load_data(simulation, planner_name)

    # Clean the current files for this experiment
    save_folder = figure_folder + "../analysis/" + simulation + "-" + planner_name
    try:
        shutil.rmtree(save_folder)
    except FileNotFoundError:
        pass

    random.seed(0)

    # Save an animation for all collision cases and some (randomly picked) success cases
    for e in tqdm(range(metrics["num experiments"])):
        if metrics["severe collisions"][e]:
            if metrics["passive collisions"][e]:
                scripts.visuals.animate_trajectories(experiment_data, e, simulation, planner_name, subfolder="passive_collisions/")
            else:
                scripts.visuals.animate_trajectories(experiment_data, e, simulation, planner_name, subfolder="active_collisions/")
        elif e % 25 == 0:
            scripts.visuals.animate_trajectories(experiment_data, e, simulation, planner_name, subfolder="success/")


def experimental_figures_diego(experiment, planner):
    filename = experiment + "_" + planner
    data_folder, metric_folder, figure_folder = get_folder_names()
    data_path = data_folder + filename + ".txt"

    metrics = dict()

    experiment_data = load_diego_ros_data(data_path)

def experimental_figures(experiment, planner):
    data_folder, metric_folder, figure_folder = get_folder_names()
    data, experiment_data, metrics, figure_folder = load_data(experiment, planner)

    figure_name = lambda i : f"{experiment}_{planner}_trajectories_{i}"

    margin = 1.5
    # X and Y flipped so that figures match the camera rotation
    MIN_X = -2.5 - margin
    MAX_X = 3.7 + margin
    MAX_Y = 3.5 + margin
    MIN_Y = -4.0 - margin
    skip_first = 75
    for idx, e in enumerate(experiment_data):
        fig = plt.figure()
        ax = plt.gca()

        # Plot the robot and obstacles
        scripts.visuals.plot_experimental_trajectories(ax, e["vehicle_pose"], t_start=skip_first, color='C0', text='Robot')
        for v in range(metrics['num obstacles']):
            scripts.visuals.plot_experimental_trajectories(ax, e[f"disc_{v+1}_pose"], color='C2', t_start=skip_first, radius=0.4, text=str(v+1))

        # Figure settings
        plt.xlim([MIN_X, MAX_X])
        plt.ylim([MIN_Y, MAX_Y])
        scripts.visuals.save_figure_as(fig, figure_folder, figure_name(idx), ratio=1., save_png=False)
        # plt.show()


if __name__ == '__main__':
    run_what = sys.argv[1]
    reload_cache = True # Set to true to not use the cache and load new data

    if run_what == "plot":  # Make a GIF of a particular experiment
        assert len(sys.argv) > 3
        simulation = sys.argv[2]
        method = sys.argv[3]

        plot_trajectories(simulation, method)
    if run_what == "experimental_figures_diego":  # Make a GIF of a particular experiment
        assert len(sys.argv) > 3
        experiment = sys.argv[2]
        planner = sys.argv[3]
        experimental_figures_diego(experiment, planner)
    if run_what == "experimental_figures":  # Make a GIF of a particular experiment
        assert len(sys.argv) > 3
        experiment = sys.argv[2]
        planner = sys.argv[3]
        experimental_figures(experiment, planner)

    elif run_what == "debug":
        simulation = sys.argv[2]
        planner_name = sys.argv[3]
        debug_visual(simulation, planner_name)
    elif run_what == "deterministic_boxplots":
        simulations = ["random_fast-4_straight", "random_fast-8_straight", "random_fast-12_straight", "random_fast-16_straight"]
        planners = ["frenet-planner", "LMPCC", "GMPCCNO", "GMPCC"]
        boxplot_of_multiple_simulations_and_planners(simulations, planners, "deterministic")
    elif run_what == "uncertain_boxplots":
        simulations = ["1-random_fast-12_straight", "01-random_fast-12_straight", "001-random_fast-12_straight"]
        planners = ["LMPCC", "GMPCC"]
        boxplot_of_multiple_simulations_and_planners(simulations, planners, "uncertain")
    elif run_what == "social_boxplots":
        simulations = ["random_social-12_straight", "random_social-16_straight"]
        planners = ["LMPCC", "GMPCCNO", "GMPCC"]
        boxplot_of_multiple_simulations_and_planners(simulations, planners, "social", reload_cache=reload_cache)
    elif run_what == "fully_social_boxplots":
        simulations = ["fully-random_social-12_straight", "fully-random_social-16_straight"]
        planners = ["LMPCC", "GMPCCNO", "GMPCC"]
        boxplot_of_multiple_simulations_and_planners(simulations, planners, "fully_social")
    elif run_what == "pedsim_notinteractive_boxplots":
        simulations = ["random_social-16_corridor"]
        planners = ["LMPCC", "GMPCCNO", "GMPCC"]
        boxplot_of_multiple_simulations_and_planners(simulations, planners, "pedsim_notinteractive", True)
    elif run_what == "pedsim_interactive_boxplots":
        simulations = ["interactive_random_social-4_corridor", "interactive_random_social-8_corridor", "interactive_random_social-12_corridor"]
        planners = ["LMPCC", "GMPCCNO", "GMPCC","Guidance-MPCCNO", "frenet-planner"]
        boxplot_of_multiple_simulations_and_planners(simulations, planners, "pedsim_interactive", False)
    elif run_what == "analyze_intrusion":
        simulations = ["random_social-16_corridor"]
        planners = ["LMPCC", "GMPCCNO", "GMPCC"]
        visualize_intrusion(simulations, planners, "notinteractive")
