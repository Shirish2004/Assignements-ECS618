"""
Author@shirish
Date@2025-09-07
Description: Implementation of the map of the problem statement.

"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patches


class Map:
    """
    Map definition
    """

    def __init__(self):
        # Map is square 30 x 30
        self.width = 30
        self.height = 30

        self.start = np.array([1.0, 1.0])
        self.goal = np.array([20.0, 20.0])
        # Obstacles defined as [x_center, y_center, radius]
        self.obstacles = [
            [4.5, 3.0, 2.0],  # Obstacle 1
            [3.0, 12.0, 2.0],  # Obstacle 2
            [15.0, 15.0, 3.0],  # Obstacle 3
        ]

    def __str__(self):
        return (
            f"Map from (0,0) to ({self.width},{self.height})\n"
            f"Start: {self.start}, Goal: {self.goal}\n"
            f"Obstacles: {len(self.obstacles)}"
        )

    def collision_check(self, point):
        """
        Checks if a given point is in collision with any obstacle.
        Also checks if point is outside map boundaries.
        """
        # Check map boundaries
        if (
            point[0] < 0
            or point[0] > self.width
            or point[1] < 0
            or point[1] > self.height
        ):
            return True

        # Check collision with obstacles
        for obs in self.obstacles:
            center = np.array([obs[0], obs[1]])
            radius = obs[2]

            # Calculate distance from point to obstacle center
            distance = np.linalg.norm(point - center)

            # Check if point is inside obstacle (with small safety margin)
            if distance <= radius:
                return True

        return False

    def visualise_map(self, path=None):
        """
        Draws the map, obstacles, start/goal points, and an optional path.
        """
        fig, ax = plt.subplots(figsize=(10, 10))

        # Draw obstacles
        for i, obs in enumerate(self.obstacles):
            circle = patches.Circle(
                (obs[0], obs[1]),
                obs[2],
                facecolor="gray",
                edgecolor="black",
                label=f"Obstacle {i + 1}",
            )
            ax.add_patch(circle)

        # Draw start and goal
        ax.plot(self.start[0], self.start[1], "b*", markersize=15, label="Start")
        ax.plot(
            self.goal[0],
            self.goal[1],
            "go",
            markersize=15,
            label="Goal",
            markerfacecolor="none",
        )

        # Draw path if one is provided
        if path and len(path) > 0:
            path_points = np.array(path)
            ax.plot(
                path_points[:, 0], path_points[:, 1], "r-", linewidth=2, label="Path"
            )

        # Configure plot
        ax.set_xlim(0, self.width)
        ax.set_ylim(0, self.height)
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("X-axis")
        ax.set_ylabel("Y-axis")
        ax.set_title("Path Planning in Configuration Space")
        ax.grid(True)

        # Create legend
        ax.legend()

        plt.show()

    def step_towards(self, current, target, step_size=0.5):
        """
        Move from current towards target by step_size.
        """
        direction = target - current
        distance = np.linalg.norm(direction)
        if distance == 0:
            return current
        step = direction / distance * min(step_size, distance)
        return current + step

    def calculate_path_length(self, path):
        """
        Calculate the total length of a given path.
        """
        if len(path) < 2:
            return 0.0
        length = 0.0
        for i in range(1, len(path)):
            length += np.linalg.norm(path[i] - path[i - 1])
        return length
