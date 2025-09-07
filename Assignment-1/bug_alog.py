"""
Author@shirish
Date@2025-09-07
Description: Implementation of Bug algorithms (Bug 0, Bug 1, Tangent Bug)
             for path planning in a 2D environment with circular obstacles.
Considering the goal and start positions from the assignment pdf, I have assumed that reaching within
goal_threshold distance from the goal is considered as reaching the goal.

"""

from map import Map
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import time
import random

random.seed(42)


class BugAlgorithms:
    def __init__(self, map_obj, step_size=0.5, max_steps=10000):
        self.map = map_obj
        self.step_size = step_size
        self.max_steps = max_steps

    def bug0(self):
        print("Running Bug0....")
        path = [self.map.start.copy()]
        position = self.map.start.copy()
        success = False
        for _ in range(self.max_steps):
            direction = self.map.goal - position
            next_pos = self.map.step_towards(position, self.map.goal, self.step_size)

            if not self.map.collision_check(next_pos):
                position = next_pos
                path.append(position.copy())
                if np.linalg.norm(position - self.map.goal) < self.step_size:
                    print("Goal reached with Bug0!")
                    success = True
                    return path, success
            else:
                tangent_direction = np.array(
                    [-direction[1], direction[0]]
                )  # perpendicular
                next_pos = self.map.step_towards(
                    position, position + tangent_direction, self.step_size
                )

                if not self.map.collision_check(next_pos):
                    position = next_pos
                    path.append(position.copy())
                else:
                    print("Trapped in obstacle, Bug0 failed.")
                    return path
        print("Max steps reached, Bug0 failed.")
        return path, success

    def bug1(self):
        print("Running Bug1....")
        path = [self.map.start.copy()]
        position = self.map.start.copy()
        hit_point = None
        leave_point = None
        boundary = []
        success = False
        for _ in range(self.max_steps):
            direction = self.map.goal - position
            next_pos = self.map.step_towards(position, self.map.goal, self.step_size)

            if not self.map.collision_check(next_pos):
                position = next_pos
                path.append(position.copy())
                if np.linalg.norm(position - self.map.goal) < self.step_size:
                    print("Goal reached with Bug1!")
                    success = True
                    return path, success
            else:
                if hit_point is None:
                    hit_point = position.copy()
                    boundary = [hit_point.copy()]
                tangent_direction = np.array([-direction[1], direction[0]])
                next_pos = self.map.step_towards(
                    position, position + tangent_direction, self.step_size
                )
                if not self.map.collision_check(next_pos):
                    position = next_pos
                    boundary.append(position.copy())
                else:
                    tangent_direction = -tangent_direction
                    next_pos = self.map.step_towards(
                        position, position + tangent_direction, self.step_size
                    )
                    if not self.map.collision_check(next_pos):
                        position = next_pos
                        boundary.append(position.copy())
                if (
                    len(boundary) > 20
                    and np.linalg.norm(position - hit_point) < self.step_size
                ):
                    leave_point = min(
                        boundary, key=lambda p: np.linalg.norm(p - self.map.goal)
                    )
                    position = leave_point.copy()
                    path.append(position.copy())
                    hit_point = None
                    boundary = []
        print("Max steps reached, Bug1 failed.")
        return path, success

    def bug_tangent(self):
        print("Running Bug Tangent....")
        success = False
        path = [self.map.start.copy()]
        position = self.map.start.copy()
        for _ in range(self.max_steps):
            direction = self.map.goal - position
            next_pos = self.map.step_towards(position, self.map.goal, self.step_size)
            if not self.map.collision_check(next_pos):
                position = next_pos
                path.append(position.copy())
                if np.linalg.norm(position - self.map.goal) < self.step_size:
                    print("Goal reached with Bug Tangent!")
                    success = True
                    return path, success
            else:
                tangent_direction = np.array([-direction[1], direction[0]])
                next_pos = self.map.step_towards(
                    position, position + tangent_direction, self.step_size
                )
                if not self.map.collision_check(next_pos):
                    position = next_pos
                    path.append(position.copy())
                else:
                    tangent_direction = -tangent_direction
                    next_pos = self.map.step_towards(
                        position, position + tangent_direction, self.step_size
                    )
                    if not self.map.collision_check(next_pos):
                        position = next_pos
                        path.append(position.copy())
                    else:
                        print("Trapped, Bug Tangent failed.")
                        return path, success
        print("Max steps reached, Bug Tangent failed.")
        return path, success

    def results(self):
        """
        Executes the planning algorithm, times it, and returns the results.
        """
        Algorithms = {
            "Bug0": self.bug0,
            "Bug1": self.bug1,
            "BugTangent": self.bug_tangent,
        }
        path_length = {"Bug0": [], "Bug1": [], "BugTangent": []}
        paths = {"Bug0": [], "Bug1": [], "BugTangent": []}
        alg0_success = {"Bug0": False, "Bug1": False, "BugTangent": False}
        algo_time = {"Bug0": 0, "Bug1": 0, "BugTangent": 0}
        for algo in Algorithms:
            start_time = time.time()
            paths[algo], alg0_success[algo] = Algorithms[algo]()
            end_time = time.time()
            algo_time[algo] = end_time - start_time
            path_length[algo] = self.map.calculate_path_length(paths[algo])
            print(f"{algo} finished in {algo_time[algo]:.4f} seconds.")
        return [Algorithms, paths, alg0_success, algo_time, path_length]
