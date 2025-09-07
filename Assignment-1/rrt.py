import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import random
from map import Map

import time


class RRT:
    def __init__(self, map_obj, step_size=1.0, max_iterations=5000, goal_threshold=1.0):
        self.map = map_obj
        self.step_size = step_size
        self.max_iterations = max_iterations
        self.goal_threshold = goal_threshold
        self.nodes = []
        self.parents = {}
        self.execution_time = 1e6

    def run(self):
        print("Running RRT...")
        self.nodes = [self.map.start.copy()]
        self.parents = {0: None}
        start_time = time.time()
        for i in range(self.max_iterations):
            rand_point = self.sample_free()
            nearest_idx = self.get_nearest_node(rand_point)
            nearest_node = self.nodes[nearest_idx]
            new_node = self.steer(nearest_node, rand_point)

            if not self.map.collision_check(new_node) and self.is_edge_free(
                nearest_node, new_node
            ):
                self.nodes.append(new_node)
                new_idx = len(self.nodes) - 1
                self.parents[new_idx] = nearest_idx

                if np.linalg.norm(new_node - self.map.goal) < self.goal_threshold:
                    print("Goal reached with RRT!")
                    self.execution_time = time.time() - start_time
                    return self.reconstruct_path(new_idx), True, self.execution_time
        print("Max iterations reached, RRT failed.")
        return [], False, self.execution_time

    def sample_free(self):
        while True:
            point = np.random.uniform([0, 0], [self.map.width, self.map.height])
            if not self.map.collision_check(point):
                return point

    def get_nearest_node(self, point):
        distances = [np.linalg.norm(node - point) for node in self.nodes]
        return np.argmin(distances)

    def steer(self, from_node, to_point):
        direction = to_point - from_node
        distance = np.linalg.norm(direction)
        if distance < self.step_size:
            return to_point
        direction = direction / distance
        return from_node + direction * self.step_size

    def is_edge_free(self, from_node, to_node, step_size=0.5):
        direction = to_node - from_node
        distance = np.linalg.norm(direction)
        if distance == 0:
            return True
        direction = direction / distance
        steps = int(distance / step_size)
        for i in range(1, steps + 1):
            point = from_node + direction * (i * step_size)
            if self.map.collision_check(point):
                return False
        return True

    def reconstruct_path(self, node_idx):
        path = []
        while node_idx is not None:
            path.append(self.nodes[node_idx])
            node_idx = self.parents[node_idx]
        path.reverse()
        return path

    def visualize(self, path, success):
        fig, ax = plt.subplots(figsize=(10, 10))
        for i, obs in enumerate(self.map.obstacles):
            circle = patches.Circle(
                (obs[0], obs[1]),
                obs[2],
                facecolor="gray",
                edgecolor="black",
                label=f"Obstacle {i + 1}",
            )
            ax.add_patch(circle)
        for idx, parent_idx in self.parents.items():
            if parent_idx is not None:
                p1 = self.nodes[idx]
                p2 = self.nodes[parent_idx]
                ax.plot([p1[0], p2[0]], [p1[1], p2[1]], "y--", linewidth=0.5)
        ax.plot(
            self.map.start[0], self.map.start[1], "b*", markersize=15, label="Start"
        )
        ax.plot(
            self.map.goal[0],
            self.map.goal[1],
            "go",
            markersize=15,
            label="Goal",
            markerfacecolor="none",
        )
        if len(path) > 0:
            path_points = np.array(path)
            linestyle = "-" if success else "--"
            color = "red" if success else "gray"
            ax.plot(
                path_points[:, 0],
                path_points[:, 1],
                linestyle,
                color=color,
                linewidth=2,
                label=f"RRT {'Success' if success else 'Failed'}",
            )
        ax.set_xlim(0, self.map.width)
        ax.set_ylim(0, self.map.height)
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("X-axis")
        ax.set_ylabel("Y-axis")
        ax.set_title(f"RRT Path Planning ({'Success' if success else 'Failed'})")
        ax.grid(True)
        ax.legend()
        plt.show()
