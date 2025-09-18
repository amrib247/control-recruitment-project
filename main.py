import numpy as np
import math
from simulator import Simulator, centerline

sim = Simulator()

def controller(x):
    """controller for a car

    Args:
        x (ndarray): numpy array of shape (5,) containing [x, y, heading, velocity, steering angle]

    Returns:
        ndarray: numpy array of shape (2,) containing [fwd acceleration, steering rate]
    """
    offset1 = 5
    offset2 = 5

    cone = closest_cone(x)
    point = cone + np.array([offset1, offset2])
    angle_to_cone = math.atan((point[0] - x[0])/(point[1] - x[1]))
    return np.array([11.99, -math.tanh(x[2] - angle_to_cone)])

def closest_cone(x):
        return get_closest_cones(x, 1)[0]

def get_distance(x, cone):
    return (x[0] - cone[0]) ** 2 + (x[1] - cone[1]) ** 2

def get_distances(x, arr):
    ret = np.full(len(arr), 0.)
    for i in range(len(arr)):
        ret[i] = get_distance(x, arr[i])
    return ret

def get_closest_cones(x, k):

    cones = sim.cones
    #print(cones)
    closest = np.full((k, 2), np.full(2, math.inf))
    for cone in cones: 
        #print(cone[0])
        if get_distance(x, cone) < min(get_distances(x, closest)):
            max_idx = 0
            for i in range(len(closest)):
                if get_distance(x, closest[max_idx]) < get_distance(x, closest[i]):
                    max_idx = 1
            closest[i] = cone

    return closest
    
sim.set_controller(controller)
sim.run()
sim.animate()
sim.plot()