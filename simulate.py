import numpy as np
import matplotlib.pyplot as plt
import system_params
import drone_controls

A = system_params.A
B = system_params.B
unsafe_zones = system_params.unsafe_zones
unsafe_zones_drawings = system_params.unsafe_zones_drawings
target_zone = system_params.target_zone

#####  Simulate System  #############################

# starting position and control input
x = np.array([0, 0, 0, 0])  # [x_pos, x_vel, y_pos, y_vel]
u = drone_controls.update_acceleration_towards_target(x, target_zone) # [x_acceleration, y_acceleration]

# for plotting purposes
trajectory = [x.copy()]

# drone in unsafe_zone checker
def in_unsafe_zone(pos):
    for zone in unsafe_zones:
        if zone[0][0] <= pos[0] <= zone[0][1] and zone[1][0] <= pos[1] <= zone[1][1]:
            return True
    return False

for step in range(100):
    x = A @ x + B @ u
    trajectory.append(x.copy())
    
    u = drone_controls.test_acceleration_and_avoidance(x, target_zone, unsafe_zones)

    print(f"Step {step + 1}: x_pos={x[0]:.4f}, x_vel={x[1]:.4f}, y_pos={x[2]:.4f}, y_vel={x[3]:.4f}")

trajectory = np.array(trajectory)

#####################################################

#####  Plot Results #################################
fig, ax = plt.subplots(figsize=(6, 6))

ax.set_xlim(-5, 10) # 0, 5
ax.set_ylim(-5, 10) # 0, 5
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
