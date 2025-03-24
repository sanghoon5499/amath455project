import numpy as np
import matplotlib.pyplot as plt
import random

##### System parameters #####
tau = 0.1
A = np.array(
    [[1, tau, 0, 0],
     [0, 1, 0, 0],
     [0, 0, 1, tau],
     [0, 0, 0, 1]]
)

B = np.array(
    [[0.5 * tau**2, 0],
     [tau, 0],
     [0, 0.5 * tau**2],
     [0, tau]]
)

unsafe_zones = [
    # obstacles:
    ([1, 2], [1, 2]),
    ([1, 2], [3, 5]),
    ([3, 4], [1, 2]),
    ([3, 4], [2.5, 3.5]),

    # walls:
    ([-0.1, 0], [0, 5]),
    ([5, 5.1], [0, 5]),
    ([0, 5], [-0.1, 0]),
    ([0, 5], [5, 5.1]),
]

target_zone = ([4, 5], [4, 5])

# RRT parameters
x0 = np.array([0.5, 0, 0.5, 0])  # Start state
max_iters = 10000
step_size = 0.5

class Node:
    def __init__(self, state, parent=None):
        self.state = state
        self.parent = parent
        self.u = None  # Control input

def sample_free():
    """Randomly sample a state within the boundaries."""
    x = np.random.uniform(0, 5)
    y = np.random.uniform(0, 5)
    return np.array([x, 0, y, 0])

def nearest(tree, sample):
    """Find the nearest node in the tree to the sample."""
    return min(tree, key=lambda node: np.linalg.norm(node.state[[0,2]] - sample[[0,2]]))

def is_collision_free(x1, x2):
    """Check if the path between x1 and x2 is collision-free."""
    for zone in unsafe_zones:
        if min(x1[0], x2[0]) < zone[0][1] and max(x1[0], x2[0]) > zone[0][0] and \
           min(x1[2], x2[2]) < zone[1][1] and max(x1[2], x2[2]) > zone[1][0]:
            return False
    return True

def steer(x_nearest, x_rand):
    """Compute new state using system dynamics x_new = Ax + Bu."""
    u = np.clip((x_rand[[0,2]] - x_nearest[[0,2]]) / step_size, -1, 1)
    x_new = A @ x_nearest + B @ u
    return (x_new, u) if is_collision_free(x_nearest, x_new) else (None, None)

# RRT Algorithm
tree = [Node(x0)]
for _ in range(max_iters):
    x_rand = sample_free()
    nearest_node = nearest(tree, x_rand)
    x_new, u_new = steer(nearest_node.state, x_rand)
    if x_new is not None:
        new_node = Node(x_new, nearest_node)
        new_node.u = u_new
        tree.append(new_node)
        if target_zone[0][0] <= x_new[0] <= target_zone[0][1] and target_zone[1][0] <= x_new[2] <= target_zone[1][1]:
            goal_node = new_node
            break
else:
    goal_node = None

# Reconstruct path
path = []
if goal_node:
    node = goal_node
    while node:
        path.append(node.state)
        node = node.parent
    path.reverse()

# Plot results
fig, ax = plt.subplots(figsize=(6, 6))
ax.set_xlim(0, 5)
ax.set_ylim(0, 5)
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_title("RRT Path with Dynamics")

# Plot obstacles
for zone in unsafe_zones:
    rect = plt.Rectangle((zone[0][0], zone[1][0]), zone[0][1]-zone[0][0], zone[1][1]-zone[1][0],
                         color="red", alpha=0.5)
    ax.add_patch(rect)

# Plot target zone
rect = plt.Rectangle((target_zone[0][0], target_zone[1][0]), 
                     target_zone[0][1] - target_zone[0][0], target_zone[1][1] - target_zone[1][0],
                     color="blue", alpha=0.5)
ax.add_patch(rect)

# Plot tree
for node in tree:
    if node.parent:
        ax.plot([node.state[0], node.parent.state[0]], [node.state[2], node.parent.state[2]], "g.-")

# Plot path
if path:
    path = np.array(path)
    ax.plot(path[:, 0], path[:, 2], "b.-", label="Planned Path")

ax.legend()
plt.grid()
plt.show()
