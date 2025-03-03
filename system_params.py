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

target_zone = ([4, 5], [4, 5])