import numpy as np
import matplotlib.pyplot as plt
import drone_controls

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
    ([1, 2], [1, 2]),
    ([1, 2], [3, 5]),
    ([3, 4], [1, 2]),
    ([3, 4], [2.5, 3.5])
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
u = np.array([0.1, 0.1]) # drone_controls.update_acceleration_towards_target(x, target_zone)

# for plotting purposes
trajectory = [x.copy()]

# drone near unsafe_zone checker
def near_unsafe_zone(pos):
    for zone in unsafe_zones:
        radius = 0.5
        if zone[0][0]-radius <= pos[0] <= zone[0][1]+radius and zone[1][0]-radius <= pos[2] <= zone[1][1]+radius:
            return True
    return False

def in_target_zone(pos):
    if target_zone[0][0] <= pos[0] <= target_zone[0][1] and target_zone[1][0] <= pos[2] <= target_zone[1][1]:
        return True
    return False

def go_to_target(pos):
    target_center = np.array([(target_zone[0][0] + target_zone[0][1]) / 2,
                              (target_zone[1][0] + target_zone[1][1]) / 2])
    
    position = np.array([pos[0], pos[2]])
    direction = target_center - position
    distance = np.linalg.norm(direction)
    
    # Normalize direction for unit vector
    if distance > 0:
        direction = direction / distance
    
    # Acceleration magnitude
    acc_magnitude = max(0.1, min(1.0, distance / 5))
    u = direction * acc_magnitude
    
    return u

def avoid_direction(pos):
    avoidance_dir = ""
    for zone in unsafe_zones:
        if zone[0][0] - 0.5 <= pos[0] <= zone[0][1] + 0.5 and zone[1][0] - 0.5 <= pos[2] <= zone[1][1] + 0.5:
            if pos[0] < zone[0][0]:  # Obstacle is to the right
                avoidance_dir = "OBSTACLE RIGHT"
            elif pos[0] > zone[0][1]:  # Obstacle is to the left
                avoidance_dir = "OBSTACLE LEFT"
            if pos[1] < zone[1][0]:  # Obstacle is above
                avoidance_dir = "OBSTACLE ABOVE"
            elif pos[1] > zone[1][1]:  # Obstacle is below
                avoidance_dir = "OBSTACLE BELOW"
    
    return avoidance_dir

for step in range(100):
    x = A @ x + B @ u
    trajectory.append(x.copy())

    print(f"In target zone: {in_target_zone(x)}, Near unsafe zone: {near_unsafe_zone(x)}")

    # accelerate towards target until max speed reached
    if not in_target_zone(x) and not near_unsafe_zone(x):
        print("TO TARGET")
        u = go_to_target(x) #np.array([u[0]*1.2, u[1]*1.2])
    else:
        print("MAINTAIN SPEED")
        u = np.array([0, 0])

    if (near_unsafe_zone(x)):
        print("NEAR UNSAFE ZONE")
        print(f"Avoid direction: {avoid_direction(x)}")
        

    if (in_target_zone(x)):
        print("IN TARGET ZONE")
        u = np.array([x[1]*-1.5, x[3]*-1.5])

    print("target direction: ", go_to_target(x))
    #print(f"Step {step + 1}: x_acc={u[0]:.4f}, y_acc={u[1]:.4f}")
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
