from glob import glob
import time
import math
import random
import numpy as np
import scipy.stats as stats
import matplotlib.pyplot as plt

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


robot_state = RobotDecisionState.movement
rospy.init_node('vector_controller', anonymous=True)
odom_sub = rospy.Subscriber("/odom", Odometry, new_odometry)
laser_sub = rospy.Subscriber('/vector/laser', Range, laser_callback)
velocity_publisher = rospy.Publisher('/vector/cmd_vel', Twist, queue_size=10)

command_initial_position = robot_position.copy()
translate_distance = 0
rotation_angle = 0

sensor_min_val = 100 # TODO: What is that?

map = Map(MAP_PATH)
min_x, max_x, min_y, max_y = map.boundary()
particles = np.empty((PARTICLE_COUNT, 3))
particles[:, 0] = np.random.uniform(min_x, max_x, size=PARTICLE_COUNT) + map.global_map_poses[0]
particles[:, 1] = np.random.uniform(min_x, max_x, size=PARTICLE_COUNT) + map.global_map_poses[1]
particles[:, 2] = np.random.choice([-90, 90, 180, 0], size=PARTICLE_COUNT) * math.pi / 180.0


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
        particles[i][2] += angle
        # TODO: Add normal noise model
        while particles[i][2] > math.pi:
            particles[i][2] -= 2 * math.pi
        while particles[i][2] < -math.pi:
            particles[i][2] += 2 * math.pi

def move_particles(distance):
    global particles
    for i in range(len(particles)):
        dx = distance * math.cos(particles[i][2])
        dy = distance * math.sin(particles[i][2])
        # TODO: Add normal noise model
        particles[i][0] += dx
        particles[i][1] += dy

def robot_extra_check():
    global sensor_min_val, command_initial_position, robot_state, state_after_stop
    sensor_min_val = sensor_range
    if EXTRA_ANGLE_CHECK and sensor_range > 1.5 * translate_distance:
        command_initial_position = robot_position.copy()
        robot_state = RobotDecisionState.rotating
        rotation_angle = -EXTRA_ANGLE_CHECK_ANGLE_DEG * PI / 180
        state_after_stop = RobotDecisionState.making_sure_front_is_accessible_1
        rotate_particles(rotation_angle)
    else:
        robot_state = RobotDecisionState.thinking

def front_is_accessible_1():
    global command_initial_position, robot_state, state_after_stop
    command_initial_position = robot_position.copy()
    robot_state = RobotDecisionState.rotating
    rotation_angle = 2 * EXTRA_ANGLE_CHECK_ANGLE_DEG * PI / 180
    state_after_stop = RobotDecisionState.making_sure_front_is_accessible_2
    rotate_particles(rotation_angle)

def front_is_accessible_2():
    global command_initial_position, robot_state, state_after_stop
    command_initial_position = robot_position.copy()
    robot_state = RobotDecisionState.rotating
    rotation_angle = -EXTRA_ANGLE_CHECK_ANGLE_DEG * PI / 180
    state_after_stop = RobotDecisionState.thinking
    rotate_particles(rotation_angle)

def choose_random_rotation():
    global rotation_angle
    angles_deg = [0, 90, -90, 180]
    angle_deg = random.choice(angles_deg)
    print("Rotate " + str(angle_deg) + " degree")
    rotation_angle = angle_deg * PI / 180

def choose_random_translation():
    global translate_distance
    translate_distances = [0, 0.1, 0.2, 0.3, 0.4, 0.5]
    translate_distance = random.choice(translate_distances)
    print("Translate " + str(translate_distance) + " meter")

def rotate():
    global sensor_min_val, robot_state

    rotation_final_angle = command_initial_position.theta + rotation_angle
    theta_error = normalize_angle(rotation_final_angle - robot_position.theta)

    if abs(theta_error) < ROTATION_ERROR_TOLERANCE:
        vel_msg = get_stop_vel_msg()
        velocity_publisher.publish(vel_msg)
        time.sleep(0.1)
        robot_state = RobotDecisionState.translating
        return

    vel_msg = get_stop_vel_msg()
    vel_msg.angular.z = sign(theta_error) * min(ROTATION_SPEED_MAX, abs(theta_error) * ROTATION_ERROR_TO_VELOCITY_COEF)
    velocity_publisher.publish(vel_msg)

def translate():
    global robot_state, translate_distance
    translation_error = translate_distance - math.sqrt(
            math.pow(robot_position.x - command_initial_position.x, 2) + 
            math.pow(robot_position.y - command_initial_position.y, 2))

    if abs(translation_error) <= TRANSLATION_ERROR_TOLERANCE:
        vel_msg = get_stop_vel_msg()
        velocity_publisher.publish(vel_msg)
        time.sleep(0.1)
        robot_state = RobotDecisionState.updating
        return

    vel_msg = get_stop_vel_msg()
    vel_msg.linear.x = min(TRANSLATION_SPEED_MAX, translation_error * TRANSLATION_ERROR_TO_VELOCITY_COEF)
    velocity_publisher.publish(vel_msg)

def calculate_particle_weights():
    global particles
    map_lines = map.get_lines()
    prob_sum = 0
    weights = np.zeros(len(particles))
    for i in range(len(particles)):
        if map.is_invalid_point(particles[i]):
            weights[i] = 0
            continue
        min_distance = 0.4

        sensor_line = get_sensor_line(particles[i])

        for line in map_lines:
            does_intersect, intersection_point = find_intersection(sensor_line[0], sensor_line[1], line[0], line[1]) 
            distance = 0.4
            if does_intersect:
                distance = calculate_distance(particles[i], intersection_point)
                min_distance = min(min_distance, distance)

        # TODO: when no intersection, is 0.4 still correct?
        weights[i] = stats.norm(min_distance, 0.01049).pdf(sensor_range)
        prob_sum += weights[i]

    for i in range(len(particles)):
        weights[i] /= prob_sum

    return weights

def generate_random_particles(size):
    global map
    min_x, max_x, min_y, max_y = map.boundary()
    new_particles = np.empty((size, 3))
    new_particles[:, 0] = np.random.uniform(min_x, max_x, size=size) + map.global_map_poses[0]
    new_particles[:, 1] = np.random.uniform(min_y, max_y, size=size) + map.global_map_poses[1]
    new_particles[:, 2] = np.random.choice([-90, 90, 180, 0], size=size) * math.pi / 180.0
    return new_particles

def roullete_wheel_resampling(particles, weights):
    sampled_indices = np.random.choice(PARTICLE_COUNT, int(0.8 * PARTICLE_COUNT), p=weights)
    particles = particles[sampled_indices]

    new_particles_count = int(0.2 * PARTICLE_COUNT)
    new_particles = generate_random_particles(new_particles_count)
    particles = np.concatenate([particles, new_particles])
    return particles

def visualize():
    global map, robot_position
    plt.clf()               
    plt.gca().invert_yaxis()
    map.plot()
    for particle in particles:
        draw_status(particle, 'red')

    draw_status(robot_position.get_state_list(), 'blue')
    draw_sensor_line(robot_position.get_state_list())

    plt.draw()
    plt.pause(0.2)

def update():
    global robot_state, particles

    weights = calculate_particle_weights()
    particles = roullete_wheel_resampling(particles, weights)
    visualize()

def check_for_halting():
    global robot_state
    robot_state = RobotDecisionState.movement

while not rospy.is_shutdown():
    if robot_state == RobotDecisionState.movement:
        choose_random_rotation()
        choose_random_translation()
        robot_state = RobotDecisionState.rotating

    elif robot_state == RobotDecisionState.rotating:
        rotate()

    elif robot_state == RobotDecisionState.translating:
        translate()

    elif robot_state == RobotDecisionState.updating:
        rotate_particles(rotation_angle)
        move_particles(translate_distance)
        update()
        check_for_halting()
    
    elif robot_state == RobotDecisionState.halt:
        pass
