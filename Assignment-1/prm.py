import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import heapq
from map import Map
import time


class PRM:
    def __init__(self, map_obj, num_samples=200, radius=5.0):
        self.map = map_obj
        self.num_samples = num_samples
        self.radius = radius
        self.nodes = []
        self.edges = {}
        self.execution_time = 1e6

    def sample_points(self):
        self.nodes = []
        attempts = 0
        while len(self.nodes) < self.num_samples and attempts < self.num_samples * 10:
            point = np.random.uniform([0, 0], [self.map.width, self.map.height])
            if not self.map.collision_check(point):
                self.nodes.append(point)
            attempts += 1
        self.nodes.append(self.map.start.copy())
        self.nodes.append(self.map.goal.copy())

    def build_roadmap(self):
        self.edges = {i: [] for i in range(len(self.nodes))}
        for i, node_i in enumerate(self.nodes):
            for j, node_j in enumerate(self.nodes):
                if i == j:
                    continue
                if np.linalg.norm(node_i - node_j) <= self.radius:
                    if self.is_collision_free(node_i, node_j):
                        cost = np.linalg.norm(node_i - node_j)
                        self.edges[i].append((j, cost))

    def is_collision_free(self, point1, point2, step_size=0.5):
        direction = point2 - point1
        distance = np.linalg.norm(direction)
        if distance == 0:
            return True
        direction = direction / distance
        steps = int(distance / step_size)
        for i in range(1, steps + 1):
            intermediate = point1 + direction * (i * step_size)
            if self.map.collision_check(intermediate):
                return False
        return True

    def search_path(self):
        """Uses Dijkstra's algorithm to find the shortest path in the roadmap."""
        start_idx = len(self.nodes) - 2
        goal_idx = len(self.nodes) - 1
        visited = set()
        heap = [(0, start_idx, [])]
        while heap:
            cost, current, path = heapq.heappop(heap)
            if current in visited:
                continue
            visited.add(current)
            path = path + [current]
            if current == goal_idx:
                return path
            for neighbor, edge_cost in self.edges[current]:
                if neighbor not in visited:
                    heapq.heappush(heap, (cost + edge_cost, neighbor, path))
        return None

    def get_path_coordinates(self, path_indices):
        if path_indices is None:
            return []
        return [self.nodes[i] for i in path_indices]

    def run(self):
        print("Running PRM...")
        start_time = time.time()
        self.sample_points()
        self.build_roadmap()
        path_indices = self.search_path()
        if path_indices is None:
            print("No path found!")
            end_time = time.time()
            self.execution_time = end_time - start_time
            return [], False, self.execution_time
        else:
            self.execution_time = time.time() - start_time
            print("Path found!")
            return self.get_path_coordinates(path_indices), True, self.execution_time

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
        nodes = np.array(self.nodes)
        ax.scatter(nodes[:, 0], nodes[:, 1], c="orange", s=10, label="Sampled Points")
        for i, neighbors in self.edges.items():
            for j, _ in neighbors:
                n1 = self.nodes[i]
                n2 = self.nodes[j]
                ax.plot([n1[0], n2[0]], [n1[1], n2[1]], "y--", linewidth=0.5)
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
                label=f"PRM {'Success' if success else 'Failed'}",
            )
        ax.set_xlim(0, self.map.width)
        ax.set_ylim(0, self.map.height)
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("X-axis")
        ax.set_ylabel("Y-axis")
        ax.set_title(f"PRM Path Planning ({'Success' if success else 'Failed'})")
        ax.grid(True)
        ax.legend()
        plt.show()
