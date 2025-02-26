import numpy as np
import matplotlib.pyplot as plt

#####  System parameters  ###########################
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

"""
I cheated a little bit by increasing the unsafe_zone borders, or else the drone flies right into them.
    Since it takes a few steps for the velocity to become negative again, there's a considerable amount of time
    the drone remains in the unsafe_zone.
"""
unsafe_zones = [
    ([0.75, 2.25], [0.75, 2.25]),
    ([0.75, 2.25], [2.75, 5.25]),
    ([2.75, 4.25], [0.75, 2.25]),
    ([2.75, 4.25], [2.25, 3.75])
]

unsafe_zones_drawings = [
    ([1, 2], [1, 2]),
    ([1, 2], [3, 5]),
    ([3, 4], [1, 2]),
    ([3, 4], [2.5, 3.5])
]

#  = [
#     ([1, 2], [1, 2]),
#     ([1, 2], [3, 5]),
#     ([3, 4], [1, 2]),
#     ([3, 4], [2.5, 3.5])
# ]

target_zone = ([4, 5], [4, 5])

#####################################################

#####  Simulate System  #############################

# starting position and control input
x = np.array([0, 0, 0, 0])  # [x_pos, x_vel, y_pos, y_vel]
u = np.array([0.25, 0.25])  # [x_acceleration, y_acceleration]

# for plotting purposes
trajectory = [x.copy()]

# drone in unsafe_zone checker
def in_unsafe_zone(pos):
    for zone in unsafe_zones:
        if zone[0][0] <= pos[0] <= zone[0][1] and zone[1][0] <= pos[1] <= zone[1][1]:
            return True
    return False

avoid_counter = 0

def turn_drone(u):
    """
    Turn the UAV 90 degrees by applying a strong negative acceleration in y-direction
     - this is problematic code as it only makes the drone turn right
     - need drone-wall positioning/detecting code to determine which of x or y velocity to change
    """
    return np.array([u[0], -5*u[1]]) # np.array([u[1], -u[0]])

for step in range(100):
    x = A @ x + B @ u
    trajectory.append(x.copy())

    if avoid_counter > 0:
        avoid_counter -= 1  # continue avoiding for 10 steps
        if avoid_counter == 0:
            """
            we reset the velocity back to facing the target, but since the drone moved a little bit,
                we need to update these velocities so that they're facing the target again.

            I think we can just calculate the slope between drone and target, and use those x and y values
                and scale them down to an appropriate speed for u = np.array([run, rise])
            """
            u = np.array([0.25, 0.25])

    elif in_unsafe_zone((x[0], x[2])):
        u = turn_drone(u)
        avoid_counter = 5

    print(f"Step {step + 1}: x_pos={x[0]:.4f}, x_vel={x[1]:.4f}, y_pos={x[2]:.4f}, y_vel={x[3]:.4f}")

trajectory = np.array(trajectory)

#####################################################

#####  Plot Results #################################
fig, ax = plt.subplots(figsize=(6, 6))

ax.set_xlim(0, 5)
ax.set_ylim(0, 5)
ax.set_xlabel(r"$x_1$")
ax.set_ylabel(r"$x_3$")
ax.set_title("Reach-Avoid-Stay Path with Obstacle Avoidance")

# bbstacles
for zone in unsafe_zones_drawings:
    rect = plt.Rectangle((zone[0][0], zone[1][0]), zone[0][1]-zone[0][0], zone[1][1]-zone[1][0],
                         color="red", alpha=0.5, label="Obstacle" if "Obstacle" not in ax.get_legend_handles_labels()[1] else "")
    ax.add_patch(rect)

# target
rect = plt.Rectangle((target_zone[0][0], target_zone[1][0]),
                     target_zone[0][1] - target_zone[0][0], target_zone[1][1] - target_zone[1][0],
                     color="blue", alpha=0.5, label="Target Set")
ax.add_patch(rect)

# plot trajectory
ax.plot(trajectory[:, 0], trajectory[:, 2], marker='o', label="Trajectory", color='green')

ax.legend(loc='upper left')
plt.grid()
plt.show()
