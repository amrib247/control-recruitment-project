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
    # Gets nearest 2 cones to the right of the middle of the vehicle
    cones = get_closest_cones(x, 2, cond = lambda cone: cone[1] - x[1] < math.tan(x[2]) * (cone[0] - x[0])) 
    point = np.array([cones[0][0] - cones[1][0], cones[0][1] - cones[1][1]]) # Midpoint of 2 cones
    
    
    #print(cones)
    #angle_to_cone = math.atan((point[0] - x[0])/(point[1] - x[1]))
    return np.array([11.99, (x[2] - angle_to_cone) * -100])

def closest_cone(x):
        """Returns the single closest cone"""

        return get_closest_cones(x, 1)[0]

def get_distance(x, cone):
    """Gives the distance from the vehicle to a given cone ([x, y])"""

    return (x[0] - cone[0]) ** 2 + (x[1] - cone[1]) ** 2

def get_distances(x, arr):
    """Returns an array of distances from the car to an array of cones"""

    ret = np.full(len(arr), 0.)
    for i in range(len(arr)):
        ret[i] = get_distance(x, arr[i])
    return ret

def get_closest_cones(x, k, cond = lambda cone: True):
    """Gets the k closest cones to the car, with the optional paramater cond. Cond takes in a
    set of cone coordinates and returns a boolean whether to check the cone or not. This can be
    used to exclude cones"""

    cones = sim.cones
    closest = np.full((k, 2), np.full(2, math.inf))
    for cone in cones: 
        if cond(cone) and get_distance(x, cone) < max(get_distances(x, closest)):
            max_idx = 0
            for i in range(len(closest)):
                if get_distance(x, closest[max_idx]) < get_distance(x, closest[i]):
                    max_idx = i
            closest[max_idx] = cone
    return closest
    
sim.set_controller(controller)
sim.run()
sim.animate()
sim.plot()