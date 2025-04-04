import numpy as np
import matplotlib.pyplot as plt

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

##### RRT parameters #####
x0 = np.array([0.5, 0, 0.5, 0])
max_iters = 10000
step_size = 0.5

class Node:
    def __init__(self, state, parent=None):
        self.state = state
        self.parent = parent
        self.u = None

##### Randomly sample a new node #####
def sample_free():
    goal_bias = 0.2
    if np.random.rand() < goal_bias:
        x = np.random.uniform(target_zone[0][0], target_zone[0][1])
        y = np.random.uniform(target_zone[1][0], target_zone[1][1])
    else:
        x = np.random.uniform(0, 5)
        y = np.random.uniform(0, 5)
    return np.array([x, 0, y, 0])

##### Find the nearest node in the RRT tree to the sampled new node #####
def nearest(tree, sample):
    return min(tree, key=lambda node: np.linalg.norm(node.state[[0,2]] - sample[[0,2]]))

##### Check for collisions #####
def is_collision_free(x1, x2):
    for zone in unsafe_zones:
        if min(x1[0], x2[0]) < zone[0][1] and max(x1[0], x2[0]) > zone[0][0] and \
           min(x1[2], x2[2]) < zone[1][1] and max(x1[2], x2[2]) > zone[1][0]:
            return False
    return True

##### x_new = Ax + Bu, control vector modified with proportional control k_p #####
def steer(x_nearest, x_rand):
    """Compute new state using system dynamics x_new = Ax + Bu."""
    K_p = 1.5
    u = np.clip(K_p * (x_rand[[0,2]] - x_nearest[[0,2]]) / step_size, -1, 1)
    x_new = A @ x_nearest + B @ u
    x_new[1] = np.clip(x_new[1], -1, 1)
    x_new[3] = np.clip(x_new[3], -1, 1)
    return (x_new, u) if is_collision_free(x_nearest, x_new) else (None, None)

##### RRT loop #####
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

##### Reconstruct the path #####
path = []
control_inputs = []
if goal_node:
    node = goal_node
    while node:
        path.append(node.state)
        if node.u is not None:
            control_inputs.append(node.u)
        node = node.parent
    path.reverse()
    control_inputs.reverse()

##### Grid set up #####
fig, ax = plt.subplots(figsize=(6, 6))
ax.set_xlim(0, 5)
ax.set_ylim(0, 5)
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_title("RRT Path")

##### Plot obstacle zones #####
for zone in unsafe_zones:
    rect = plt.Rectangle((zone[0][0], zone[1][0]), zone[0][1]-zone[0][0], zone[1][1]-zone[1][0],
                         color="red", alpha=0.5)
    ax.add_patch(rect)

##### Plot target zone #####
rect = plt.Rectangle((target_zone[0][0], target_zone[1][0]), 
                     target_zone[0][1] - target_zone[0][0], target_zone[1][1] - target_zone[1][0],
                     color="blue", alpha=0.5)
ax.add_patch(rect)

##### Plot RRT path #####
for node in tree:
    if node.parent:
        ax.plot([node.state[0], node.parent.state[0]], [node.state[2], node.parent.state[2]], "g.-")

if path:
    path = np.array(path)
    ax.plot(path[:, 0], path[:, 2], "b.-", label="Planned Path")

ax.legend(loc='lower right')
plt.grid()
plt.show()

##### Print the list of control vectors #####
print("Control Inputs:", control_inputs)
