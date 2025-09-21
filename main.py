import numpy as np
import math
from simulator import Simulator, centerline

sim = Simulator()

#distance = 0.0

# Load in centerline as an array of points
num_points = 1000
track_length = 104.9
centerline_points = np.full((num_points, 2), np.array([0., 0.]))
for i in range(num_points):
    centerline_points[i] = centerline(track_length * i / num_points)

previous_angle = 0.0


def controller(x):
    """controller for a car

    Args:
        x (ndarray): numpy array of shape (5,) containing [x, y, heading, velocity, steering angle]

    Returns:
        ndarray: numpy array of shape (2,) containing [fwd acceleration, steering rate]
    """
    """
    turn_scale = 1
    distance_from_cone = 2.5
    # Gets nearest 2 cones to the right of the middle of the vehicle
    cones = get_closest_cones(x, 2, cond = lambda cone: cone[1] - x[1] < math.tan(x[2]) * (cone[0] - x[0])) 
    point = np.array([cones[0][0] - cones[1][0], cones[0][1] - cones[1][1]]) # Midpoint of 2 cones
    #print(cones)
    angle_to_cone = math.atan((point[0] - x[0])/(point[1] - x[1]))
    angle_from_cone = angle_to_cone - math.pi/2
    point += np.array([math.cos(angle_from_cone), math.sin(angle_from_cone)]) * distance_from_cone
    angle_to_point = math.atan((point[0] - x[0])/(point[1] - x[1]))
    return np.array([11.99, math.tanh((x[2] - angle_to_point) * -turn_scale)])"""

    global centerline_points
    global previous_angle
    lead = 60


    closest = closest_centerline_point(x)
    centerline_point = centerline_points[(closest + lead) % num_points]
    angle_to_centerline = math.atan((centerline_point[1] - x[1]) / (centerline_point[0] - x[0]))
    if centerline_point[0] - x[0] < 0:
        angle_to_centerline += math.pi

    proportional_term = ((x[2] + x[4]) - angle_to_centerline) % (2 * math.pi)
    if proportional_term > math.pi:
        proportional_term -= 2 * math.pi
    proportional_term *= -0.5


    derivative_steering_angle = previous_angle - ((x[2] + x[4]) % (2 * math.pi))
    derivative_term = (previous_angle - ((x[2] + x[4]) - angle_to_centerline))  % (2 * math.pi) 
    if derivative_term > math.pi:
        derivative_term -= 2 * math.pi
    derivative_term *= 50.0

    #print("P:", proportional_term)
    #print('D', derivative_term)

    turn_scale = 3
    turn_max = 1    

    turn_command = math.tanh(((proportional_term + derivative_term)) * turn_scale) * turn_max
    previous_angle = ((x[2] + x[4]) - angle_to_centerline) % (2 * math.pi)

    #print('Angle to centerline:', angle_to_centerline)
    #print('Heading:', x[2])
    #print('Command', turn_command)
    
    velocity_max_scale = 0.0
    return np.array([(12 - velocity_max_scale * x[3]) - 0.1 * abs(derivative_steering_angle), turn_command])

def closest_centerline_point(x):
    """Returns the index of the closest point along the centerline"""
    
    nearest_idx = 0
    for i in range(num_points):
        if get_distance(x, centerline_points[i]) < get_distance(x, centerline_points[nearest_idx]):
            nearest_idx = i
    return nearest_idx

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
sim.run(15)
sim.animate()
sim.plot()