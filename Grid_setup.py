import numpy as np
import matplotlib.pyplot as plt
# import scipy.linalg

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

unsafe_zones = [
    ([1, 2], [1, 2]),
    ([1, 2], [3, 5]),
    ([3, 4], [1, 2]),
    ([3, 4], [2.5, 3.5])
]

target_zone = ([4, 5], [4, 5])

#####################################################

#####  Simulate System  #############################
# TODO

# Starting position
x0 = np.array([0.5, 0, 0.5, 0])  # [x_pos, x_vel, y_pos, y_vel]



#####################################################

#####  Plot results #################################
fig, ax = plt.subplots(figsize=(6, 6))

ax.set_xlim(0, 5)
ax.set_ylim(0, 5)
ax.set_xlabel(r"$x_1$")
ax.set_ylabel(r"$x_3$")
ax.set_title("Reach-Avoid-Stay Path")

# Obstacle boxes
for zone in unsafe_zones:
    rect = plt.Rectangle((zone[0][0], zone[1][0]), zone[0][1]-zone[0][0], zone[1][1]-zone[1][0],
                         color="red", alpha=0.5, label="Obstacle" if "Obstacle" not in ax.get_legend_handles_labels()[1] else "")
    ax.add_patch(rect)

# Target box
rect = plt.Rectangle((target_zone[0][0], target_zone[1][0]), 
                     target_zone[0][1] - target_zone[0][0], target_zone[1][1] - target_zone[1][0],
                     color="blue", alpha=0.5, label="Target Set")
ax.add_patch(rect)

# TODO: Plot trajectory



ax.legend(loc='upper left')
plt.grid()
plt.show()
