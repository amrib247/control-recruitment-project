import numpy as np
import math
from simulator import Simulator, centerline

sim = Simulator()

# Load in centerline as an array of points
num_points = 1000
track_length = 104.9
centerline_points = np.full((num_points, 2), np.array([0., 0.]))
for i in range(num_points):
    centerline_points[i] = centerline(track_length * i / num_points)

#Initialize other Global variables
previous_angle = 0.0 
time = 0.0 # Time elapsed


def controller(x):
    """controller for a car

    Args:
        x (ndarray): numpy array of shape (5,) containing [x, y, heading, velocity, steering angle]

    Returns:
        ndarray: numpy array of shape (2,) containing [fwd acceleration, steering rate]
    """
    #Setup
    global centerline_points
    global previous_angle
    global time

    lead = 60 + int(5 * ( 1 / 1 + previous_angle))

    #Time Optimization
    if x[0] < 0.2 and x[0] > -0.2 and x[1] > -1.5 and x[1] < 1.5 and time > 3:
        print('Time:', time)

    #Get Leading Centerline Point and Angle
    closest = closest_centerline_point(x)
    centerline_point = centerline_points[(closest + lead) % num_points]
    angle_to_centerline = math.atan((centerline_point[1] - x[1]) / (centerline_point[0] - x[0]))
    if centerline_point[0] - x[0] < 0:
        angle_to_centerline += math.pi

    #Calculate Proportional Response Term
    proportional_term = ((x[2] + x[4]) - angle_to_centerline) % (2 * math.pi)
    if proportional_term > math.pi:
        proportional_term -= 2 * math.pi
    proportional_term *= -(1.4 + x[3] / 46.31)

    #Calculate Derivative Response Term
    derivative_term = (previous_angle - ((x[2] + x[4]) - angle_to_centerline))  % (2 * math.pi) 
    if derivative_term > math.pi:
        derivative_term -=  2 * math.pi
    derivative_term *= 32.0

    #Calculate Final Commands and Update Global Variables
    turn_scale = 3.5
    turn_max = 1   
    velocity_max_scale = 0.0

    turn_command = math.tanh(((proportional_term + derivative_term)) * turn_scale) * turn_max
    
    previous_angle = ((x[2] + x[4]) - angle_to_centerline) % (2 * math.pi)
    if previous_angle > math.pi:
        previous_angle -= 2 * math.pi
    time += 0.01

    #Acceleration Control
    max_accel = 12
    buffer = 0
    curve_lead = 50 + int(x[3]**2 / 2.9)
    
    curve_point = centerline_points[(closest + curve_lead) % num_points]
    curve_angle = math.atan((curve_point[1] - x[1]) / (curve_point[0] - x[0]))
    if curve_point[0] - x[0] < 0:
        curve_angle += math.pi
    diff = (x[2] - curve_angle) % (2 * math.pi)
    if diff > math.pi:
        diff -= 2 * math.pi
    max_velocity_centripetal = math.sqrt(abs((max_accel - buffer) * (1.58 / max(0.0000001, math.sin(diff)))))
   
    return np.array([10 - 10 * (x[3] / max_velocity_centripetal), turn_command])


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
sim.run(90)
sim.animate()
sim.plot()