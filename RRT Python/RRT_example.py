import numpy as np
import matplotlib.pyplot as plt
import random

class RRT:
    def __init__(self, start, goal, obstacles, A, B, grid_size=(5, 5), max_iter=1000):
        self.start = np.array(start, dtype=np.float64)
        self.goal = np.array(goal, dtype=np.float64)
        self.obstacles = obstacles
        self.A = np.array(A)
        self.B = np.array(B)
        self.grid_size = grid_size
        self.max_iter = max_iter
        self.tree = {tuple(start): None}
        self.path = []

    def sample_free_space(self):
        return np.array([random.uniform(0, self.grid_size[0]), random.uniform(0, self.grid_size[1]), 0, 0])
    
    def nearest_neighbor(self, x_rand):
        return min(self.tree.keys(), key=lambda node: np.linalg.norm(np.array(node) - x_rand))
    
    def steer(self, x_nearest, x_rand, u):
        x_nearest = np.array(x_nearest)
        u = np.array(u)
        x_new = self.A @ x_nearest + self.B @ u
        return x_new
    
    def collision_free(self, x):
        x_pos = x[:2]
        for obs in self.obstacles:
            if obs[0] <= x_pos[0] <= obs[2] and obs[1] <= x_pos[1] <= obs[3]:
                return False
        return True
    
    def build_tree(self):
        for _ in range(self.max_iter):
            x_rand = self.sample_free_space()
            x_nearest = self.nearest_neighbor(x_rand)
            u = np.random.uniform(-1, 1, 2)  # Random control input
            x_new = self.steer(x_nearest, x_rand, u)
            
            if self.collision_free(x_new) and (0 <= x_new[0] <= self.grid_size[0] and 0 <= x_new[1] <= self.grid_size[1]):
                self.tree[tuple(x_new)] = x_nearest
                if np.linalg.norm(x_new[:2] - self.goal[:2]) < 0.5:
                    self.path = self.retrace_path(x_new)
                    return
    
    def retrace_path(self, x_final):
        path = [x_final]
        while path[-1] is not None:
            path.append(self.tree.get(tuple(path[-1])))
        return path[::-1]
    
    def plot(self):
        fig, ax = plt.subplots()
        ax.set_xlim(0, self.grid_size[0])
        ax.set_ylim(0, self.grid_size[1])
        
        for obs in self.obstacles:
            ax.add_patch(plt.Rectangle((obs[0], obs[1]), obs[2] - obs[0], obs[3] - obs[1], color='red'))
        
        for node in self.tree.keys():
            if self.tree[node] is not None:
                parent = np.array(self.tree[node])
                ax.plot([parent[0], node[0]], [parent[1], node[1]], 'bo-', markersize=2)
        
        if self.path:
            path_coords = np.array(self.path)
            ax.plot(path_coords[:, 0], path_coords[:, 1], 'g-', linewidth=2)
        
        ax.scatter(self.start[0], self.start[1], color='blue', s=100, label='Start')
        ax.scatter(self.goal[0], self.goal[1], color='green', s=100, label='Goal')
        ax.legend()
        plt.show()

# Define system matrices
A = np.eye(4)  # Identity for simplicity
B = np.array([[1, 0], [0, 1], [1, 0], [0, 1]])  # Control influence

# Define obstacles [(x_min, y_min, x_max, y_max)]
obstacles = [(1, 1, 2, 3), (3, 2, 4, 4)]

# Initialize and run RRT
rrt = RRT(start=(0, 0, 0, 0), goal=(4, 4, 0, 0), obstacles=obstacles, A=A, B=B)
rrt.build_tree()
rrt.plot()
