import numpy as np
import matplotlib.pyplot as plt

# drone direction updater:
def update_acceleration_towards_target(x, target_zone):
    target_center = np.array([(target_zone[0][0] + target_zone[0][1]) / 2,
                              (target_zone[1][0] + target_zone[1][1]) / 2])
    
    position = np.array([x[0], x[2]])
    direction = target_center - position
    distance = np.linalg.norm(direction)
    
    # Normalize direction for unit vector
    if distance > 0:
        direction = direction / distance
    
    # Acceleration magnitude
    acc_magnitude = max(0.1, min(1.0, distance / 5))
    u = direction * acc_magnitude
    
    return u

def turn_drone(u, acceleration):
    """
    Turn the UAV 90 degrees by applying a strong negative acceleration in y-direction
     - this is problematic code as it only makes the drone turn right
     - need drone-wall positioning/detecting code to determine which of x or y velocity to change
    """
    return np.array([u[0], u[1] - acceleration]) # np.array([u[1], -u[0]])


def test_acceleration_and_avoidance(x, target_zone, unsafe_zones):
    target_center = np.array([(target_zone[0][0] + target_zone[0][1]) / 2,
                              (target_zone[1][0] + target_zone[1][1]) / 2])
    
    position = np.array([x[0], x[2]])
    direction = target_center - position
    distance = np.linalg.norm(direction)
    
    # Normalize direction to get unit vector
    if distance > 0:
        direction = direction / distance
    
    # Reduce acceleration as the drone gets closer to the target
    acc_magnitude = max(0.1, min(1.0, distance / 5))  # Scale factor can be adjusted
    u = direction * acc_magnitude
    
    # Obstacle avoidance logic
    for zone in unsafe_zones:
        if zone[0][0] - 0.5 <= position[0] <= zone[0][1] + 0.5 and zone[1][0] - 0.5 <= position[1] <= zone[1][1] + 0.5:
            avoidance_dir = np.array([0.0, 0.0])
            if position[0] < zone[0][0]:  # Obstacle is to the right
                avoidance_dir[0] = -1
            elif position[0] > zone[0][1]:  # Obstacle is to the left
                avoidance_dir[0] = 1
            if position[1] < zone[1][0]:  # Obstacle is above
                avoidance_dir[1] = -1
            elif position[1] > zone[1][1]:  # Obstacle is below
                avoidance_dir[1] = 1
            
            # Apply a scaled avoidance force
            avoidance_magnitude = 0.5
            u += avoidance_dir * avoidance_magnitude
    
    return u