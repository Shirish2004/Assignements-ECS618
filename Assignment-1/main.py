"""
Author@shirish
Date@2025-09-07
Description: Implementation of all the algorithms and comparison of their results.
"""

from map import Map
from bug_alog import BugAlgorithms
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
from prm import PRM
from rrt import RRT
import random

random.seed(42)


def visualize_paths(map_obj, paths, success_flags):
    colors = {
        "Bug0": "red",
        "Bug1": "blue",
        "BugTangent": "purple",
        "PRM": "green",
        "RRT": "orange",
    }

    for algo in paths:
        path = paths[algo]
        success = success_flags[algo]
        fig, ax = plt.subplots(figsize=(10, 10))
        for i, obs in enumerate(map_obj.obstacles):
            circle = patches.Circle(
                (obs[0], obs[1]),
                obs[2],
                facecolor="gray",
                edgecolor="black",
                label=f"Obstacle {i + 1}",
            )
            ax.add_patch(circle)
        ax.plot(map_obj.start[0], map_obj.start[1], "b*", markersize=15, label="Start")
        ax.plot(
            map_obj.goal[0],
            map_obj.goal[1],
            "go",
            markersize=15,
            label="Goal",
            markerfacecolor="none",
        )
        if len(path) > 0:
            path_points = np.array(path)
            linestyle = "-" if success else "--"
            ax.plot(
                path_points[:, 0],
                path_points[:, 1],
                linestyle,
                color=colors[algo],
                linewidth=2,
                label=f"{algo} {'Success' if success else 'Failed'}",
            )
        ax.set_xlim(0, map_obj.width)
        ax.set_ylim(0, map_obj.height)
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("X-axis")
        ax.set_ylabel("Y-axis")
        ax.set_title(f"Path for {algo} ({'Success' if success else 'Failed'})")
        ax.grid(True)
        ax.legend()

        plt.show()


def display_results(path_length, algo_time, success_flags):
    """
    Displays the results as a pandas DataFrame.
    """
    data = {
        "Algorithm": list(path_length.keys()),
        "Path Length": [round(path_length[a], 2) for a in path_length],
        "Time (s)": [round(algo_time[a], 4) for a in algo_time],
        "Success": [success_flags[a] for a in success_flags],
    }
    df = pd.DataFrame(data)
    df.to_csv("./results.csv", index=False)

    print(df)
    return df


if __name__ == "__main__":
    my_map = Map()
    bug = BugAlgorithms(my_map)
    prm = PRM(my_map)
    rrt = RRT(my_map)
    Algorithms, paths, success_flags, algo_time, path_length = bug.results()
    prm_path, prm_success, prm_time = prm.run()
    rrt_path, rrt_success, rrt_time = rrt.run()
    prim_path_legnth = my_map.calculate_path_length(prm_path)
    rrt_path_length = my_map.calculate_path_length(rrt_path)
    paths["RRT"] = rrt_path
    success_flags["RRT"] = rrt_success
    algo_time["RRT"] = rrt_time
    path_length["RRT"] = rrt_path_length
    paths["PRM"] = prm_path
    success_flags["PRM"] = prm_success
    algo_time["PRM"] = prm_time
    path_length["PRM"] = prim_path_legnth
    rrt.visualize(rrt_path, rrt_success)
    prm.visualize(prm_path, prm_success)
    visualize_paths(my_map, paths, success_flags)

    display_results(path_length, algo_time, success_flags)
