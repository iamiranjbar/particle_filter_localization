from glob import glob
import time
import math
import random
import traceback
import numpy as np
from numpy.lib.function_base import average
import scipy.stats as stats
from threading import Thread
import matplotlib.pyplot as plt
from multiprocessing import Pool

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from tf.transformations import euler_from_quaternion

from codes.map import Map
from codes.model import RobotDecisionState, RobotWorldState
from codes.map_utils import find_intersection, calculate_distance
from codes.visualization import get_sensor_line, draw_status, draw_sensor_line


MAP_PATH = './worlds/sample1.world'

PI = math.pi

TRANSLATION_ERROR_TOLERANCE = 0.001 # One millimeters
TRANSLATION_ERROR_TO_VELOCITY_COEF = 4
TRANSLATION_SPEED_MAX = 0.1

ROTATION_ERROR_TOLERANCE = 0.1 * PI / 180
ROTATION_ERROR_TO_VELOCITY_COEF = 20
ROTATION_SPEED_MAX = 10

EXTRA_ANGLE_CHECK = False
EXTRA_ANGLE_CHECK_ANGLE_DEG = 15

ACTION_TIMEOUT = 3

PARTICLE_COUNT = 500

robot_position = RobotWorldState()
sensor_range = 0

def new_odometry(msg):
    global robot_position
 
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
 
    rot_q = msg.pose.pose.orientation
    (_, _, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    robot_position.x = x
    robot_position.y = y
    robot_position.theta = theta

def laser_callback(msg):
    global sensor_range
    sensor_range = msg.range

def generate_random_particles(size):
    global map
    min_x, max_x, min_y, max_y = map.boundary()
    new_particles = np.empty((size, 3))
    new_particles[:, 0] = np.random.uniform(min_x, max_x, size=size)
    new_particles[:, 1] = np.random.uniform(min_y, max_y, size=size)
    new_particles[:, 2] = np.random.choice([-90, 90, 180, 0], size=size) * math.pi / 180.0
    for index in range(len(new_particles)):
        while map.is_invalid_point(new_particles[index]):
            new_particles[index, 0] = np.random.uniform(min_x, max_x)
            new_particles[index, 1] = np.random.uniform(min_y, max_y)
            new_particles[index, 2] = np.random.choice([-90, 90, 180, 0]) * math.pi / 180.0
    return new_particles


robot_state = RobotDecisionState.movement
rospy.init_node('vector_controller', anonymous=True)
odom_sub = rospy.Subscriber("/odom", Odometry, new_odometry)
laser_sub = rospy.Subscriber('/vector/laser', Range, laser_callback)
velocity_publisher = rospy.Publisher('/vector/cmd_vel', Twist, queue_size=10)

command_time = 0
command_initial_position = robot_position.copy()
translate_distance = 0
rotation_angle = 0

map = Map(MAP_PATH)
particles = generate_random_particles(PARTICLE_COUNT)

estimate = []
translation_translation_variance = {0: 0, 0.05: 3.5e-5, 0.1: 3.3e-5, 0.15:3.2e-5, 0.2:3.14e-5, 0.3:2.98e-5}
translation_rotation_mean = {0: 0, 0.05: 0.03, 0.1: 0.05, 0.15: 0.1, 0.2: 0.15, 0.3: 0.19}
translation_rotation_variance = {0: 0, 0.05: 0.05, 0.1: 0.09, 0.15: 0.2, 0.2: 0.36, 0.3: 0.67}

rotation_rotation_variance = {0: 0, 90: 0.005, -90: 0.006}
rotation_translation_mean = {0: 0, 90: 0.0002, -90: 0.0001}
rotation_translation_variance = {0: 0, 90: 0.0013, -90: 0.0018}

def get_stop_vel_msg():
    vel_msg = Twist()

    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    return vel_msg
    
def sign(f):
    if f > 0:
        return 1
    elif f < 0:
        return -1
    return 0

def normalize_angle(angle):
    while angle > PI:
        angle -= 2 * PI
    while angle < -PI:
        angle += 2 * PI
    return angle

def rotate_particles(angle):
    global particles
    for i in range(len(particles)):
        angle_deg = int(angle * 180 / PI)
        rotation_variance = rotation_rotation_variance[angle_deg]
        # rotation = np.random.normal(angle, 0.005)
        rotation = np.random.normal(angle, rotation_variance)

        translation_mean = rotation_translation_mean[angle_deg]
        translation_variance = rotation_translation_variance[angle_deg]
        # translation = np.random.normal(0.0001, 0.0015)
        translation = np.random.normal(translation_mean, translation_variance)

        particles[i][2] += rotation
        particles[i][2] = normalize_angle(particles[i][2])
        particles[i][0] += translation * math.cos(angle)
        particles[i][1] += translation * math.sin(angle)

def move_particles(distance):
    global particles
    for i in range(len(particles)):
        translation_variance = translation_translation_variance[distance]
        # translation = np.random.normal(distance, 0.0025)
        translation = np.random.normal(distance, translation_variance)

        rotation_mean = translation_rotation_mean[distance]
        rotation_variance = translation_rotation_variance[distance]
        # rotation = np.random.normal(0.000123, 0.0006)
        rotation = np.random.normal(rotation_mean, rotation_variance)

        particles[i][0] += translation * math.cos(particles[i][2])
        particles[i][1] += translation * math.sin(particles[i][2])
        particles[i][2] += rotation
        particles[i][2] = normalize_angle(particles[i][2])

def robot_extra_check():
    global sensor_min_val, command_initial_position, robot_state, state_after_stop, command_time
    sensor_min_val = sensor_range
    if EXTRA_ANGLE_CHECK and sensor_range > 1.5 * translate_distance:
        command_initial_position = robot_position.copy()
        command_time = time.time()
        robot_state = RobotDecisionState.rotating
        rotation_angle = -EXTRA_ANGLE_CHECK_ANGLE_DEG * PI / 180
        state_after_stop = RobotDecisionState.making_sure_front_is_accessible_1
        rotate_particles(rotation_angle)
    else:
        robot_state = RobotDecisionState.thinking

def front_is_accessible_1():
    global command_initial_position, robot_state, state_after_stop, command_time
    command_initial_position = robot_position.copy()
    command_time = time.time()
    robot_state = RobotDecisionState.rotating
    rotation_angle = 2 * EXTRA_ANGLE_CHECK_ANGLE_DEG * PI / 180
    state_after_stop = RobotDecisionState.making_sure_front_is_accessible_2
    rotate_particles(rotation_angle)

def front_is_accessible_2():
    global command_initial_position, robot_state, state_after_stop, command_time
    command_initial_position = robot_position.copy()
    command_time = time.time()
    robot_state = RobotDecisionState.rotating
    rotation_angle = -EXTRA_ANGLE_CHECK_ANGLE_DEG * PI / 180
    state_after_stop = RobotDecisionState.thinking
    rotate_particles(rotation_angle)

def choose_random_rotation():
    global rotation_angle, robot_position, command_initial_position, command_time
    angles_deg = [0, 90, -90]
    angle_deg = random.choice(angles_deg)
    print("Rotate " + str(angle_deg) + " degree")
    rotation_angle = angle_deg * PI / 180
    command_initial_position = robot_position.copy()
    command_time = time.time()

def choose_random_translation():
    global translate_distance, sensor_range, robot_position, command_initial_position, command_time
    translate_distances = [0, 0.05, 0.1, 0.15, 0.2, 0.3]
    translate_distance = random.choice(translate_distances)
    if sensor_range < (translate_distance + 0.025):
        translate_distance = 0
    command_initial_position = robot_position.copy()
    command_time = time.time()
    print("Translate " + str(translate_distance) + " meter")

def rotate():
    global robot_state, command_initial_position

    rotation_final_angle = command_initial_position.theta + rotation_angle
    theta_error = normalize_angle(rotation_final_angle - robot_position.theta)

    if abs(theta_error) < ROTATION_ERROR_TOLERANCE or time.time() - command_time > ACTION_TIMEOUT:
        vel_msg = get_stop_vel_msg()
        velocity_publisher.publish(vel_msg)
        time.sleep(0.1)
        robot_state = RobotDecisionState.translating
        choose_random_translation()
        return

    vel_msg = get_stop_vel_msg()
    vel_msg.angular.z = sign(theta_error) * min(ROTATION_SPEED_MAX, abs(theta_error) * ROTATION_ERROR_TO_VELOCITY_COEF)
    velocity_publisher.publish(vel_msg)

def translate():
    global robot_state, translate_distance, command_initial_position, command_time
    translation_error = translate_distance - math.sqrt(
            math.pow(robot_position.x - command_initial_position.x, 2) + 
            math.pow(robot_position.y - command_initial_position.y, 2))

    if abs(translation_error) <= TRANSLATION_ERROR_TOLERANCE or time.time() - command_time > ACTION_TIMEOUT:
        vel_msg = get_stop_vel_msg()
        velocity_publisher.publish(vel_msg)
        time.sleep(0.1)
        robot_state = RobotDecisionState.updating
        return

    vel_msg = get_stop_vel_msg()
    vel_msg.linear.x = min(TRANSLATION_SPEED_MAX, translation_error * TRANSLATION_ERROR_TO_VELOCITY_COEF)
    velocity_publisher.publish(vel_msg)

map_lines = []
def calculate_particle_weight(particle):
    global map_lines, sensor_range, map
    
    has_collision = map.is_invalid_point(particle)
    if has_collision:
        return 0

    min_distance = 0.4
    sensor_line = get_sensor_line(particle)
    for line in map_lines:
        does_intersect, intersection_point = find_intersection(sensor_line[0], sensor_line[1], line[0], line[1]) 
        distance = 0.4
        if does_intersect:
            distance = calculate_distance(particle, intersection_point)
            min_distance = min(min_distance, distance)

    return stats.norm(min_distance, 0.01049).pdf(sensor_range + 0.043)

def calculate_particle_weights():
    global particles, map_lines
    map_lines = map.get_lines()
    prob_sum = 0
    weights = []
    multiprocessing_list = []
    for i in range(len(particles)):
        multiprocessing_list.append(particles[i])
    
    # print("Starting multiprocessing")
    pool = Pool(8)
    try:
        weights = np.array(pool.map(calculate_particle_weight, multiprocessing_list))
    except:
        traceback.print_exc()
    pool.close()
    pool.join()

    # print("Finished multiprocessing")
    prob_sum = np.sum(weights)
    weights /= prob_sum

    return weights

def roullete_wheel_resampling(particles, weights):
    sampled_indices = np.random.choice(PARTICLE_COUNT, int(0.8 * PARTICLE_COUNT), p=weights)
    particles = particles[sampled_indices]

    new_particles_count = int(0.2 * PARTICLE_COUNT)
    new_particles = generate_random_particles(new_particles_count)
    particles = np.concatenate([particles, new_particles])
    return particles

def best_select_resampling(particles, weights):
    best_indices = (-weights).argsort()[:int(0.3 * PARTICLE_COUNT)]
    best_particles = particles[best_indices]

    new_particles_count = int(0.2 * PARTICLE_COUNT)
    new_particles = generate_random_particles(new_particles_count)

    best_around_count = PARTICLE_COUNT - len(best_indices) - new_particles_count
    best_around_indices = np.random.choice(best_particles.shape[0], best_around_count)
    
    best_around_particles = np.empty((best_around_count, 3))
    best_around_particles[:, 0] = np.random.uniform(-0.05, 0.05, size=best_around_count)
    best_around_particles[:, 1] = np.random.uniform(-0.05, 0.05, size=best_around_count)
    best_around_particles[:, 2] = 0

    best_around_particles += best_particles[best_around_indices]
    particles = np.concatenate([best_particles, new_particles, best_around_particles])

    return particles

def visualize():
    global map, robot_position, estimate
    plt.clf()               
    plt.gca().invert_yaxis()
    map.plot()
    for particle in particles:
        draw_status(particle, 'red')

    draw_status(robot_position.get_state_list(), 'blue')
    draw_sensor_line(robot_position.get_state_list())
    if len(estimate) > 0:
        draw_status(estimate, 'green')

    plt.draw()
    plt.pause(0.2)

def get_best_particles_average_estimate(weights, verbose=False):
    global particles
    best_indices = (-weights).argsort()[:int(0.3 * PARTICLE_COUNT)]
    best_particles = particles[best_indices]
    estimate_x = average(best_particles[:, 0])
    estimate_y = average(best_particles[:, 1])
    angle_list = list(best_particles[:, 2])
    estimate_theta = max(set(angle_list), key=angle_list.count)
    estimate = np.array([estimate_x, estimate_y, estimate_theta])
    if verbose:
        print("Estimate: (" + str(estimate_x) + ", " + str(estimate_y) + ", " + str(estimate_theta) +")")

    return estimate

def get_best_particle_min_sum_distance(verbose=False):
    global particles
    distances = np.zeros(len(particles))
    for index, particle in enumerate(particles):
        distances[index] = sum(np.sqrt((particle[0] - particles[:, 0])**2 + (particle[1] - particles[:, 1])**2))
    min_sum_distance_index = np.argmin(distances)
    estimate = particles[min_sum_distance_index]
    if verbose:
        print("Estimate: (" + str(estimate[0]) + ", " + str(estimate[1]) + ", " + str(estimate[2]) +")")

    return estimate

def is_halted():
    global robot_state, estimate
    robot_state = RobotDecisionState.movement
    return False

def update():
    global robot_state, particles, rotation_angle, translate_distance, estimate

    if rotation_angle == 0 and translate_distance == 0:
        robot_state = RobotDecisionState.movement
        return

    print("Updating particles")
    t = time.time()
    weights = calculate_particle_weights()

    # estimate = get_best_particles_average_estimate(weights, verbose=True)
    estimate = get_best_particle_min_sum_distance(verbose=True)
    if is_halted():
        return

    # particles = roullete_wheel_resampling(particles, weights)
    particles = best_select_resampling(particles, weights)
    duration = time.time() - t
    print("Particles updated in " + str(duration) + "s")

def visualize_loop():
    while True:
        visualize()
        time.sleep(2)

visualize_loop_thread = Thread(target=visualize_loop)
visualize_loop_thread.start()

while not rospy.is_shutdown():
    if robot_state == RobotDecisionState.movement:
        choose_random_rotation()
        robot_state = RobotDecisionState.rotating

    elif robot_state == RobotDecisionState.rotating:
        rotate()

    elif robot_state == RobotDecisionState.translating:
        translate()

    elif robot_state == RobotDecisionState.updating:
        rotate_particles(rotation_angle)
        move_particles(translate_distance)
        update()
    
    elif robot_state == RobotDecisionState.halt:
        pass
