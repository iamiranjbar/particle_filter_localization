#!/usr/bin/env python2.7

from codes.model import RobotDecisionState, RobotWorldState
from codes.map import Map
from codes.particle import Particle
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from tf.transformations import euler_from_quaternion
import time
import math
import random

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

PARTICLE_COUNT = 1000

robot_position = RobotWorldState()
sensor_range = 0
odometry_counter = 0

def new_odometry(msg):
    global robot_position
    global odometry_counter
 
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
 
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    robot_position.x = x
    robot_position.y = y
    robot_position.theta = theta
    odometry_counter += 1

def laser_callback(msg):
    global sensor_range
    sensor_range = msg.range

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

robot_state = RobotDecisionState.initial

rospy.init_node('vector_controller', anonymous=True)
odom_sub = rospy.Subscriber("/odom", Odometry, new_odometry)
laser_sub = rospy.Subscriber('/vector/laser', Range, laser_callback)
velocity_publisher = rospy.Publisher('/vector/cmd_vel', Twist, queue_size=10)

command_initial_position = robot_position.copy()
translate_distance = 0.1
rotation_angle = 0

state_after_stop = RobotDecisionState.pre_thinking

sensor_min_val = 100

map = Map(MAP_PATH)
particles = []
for i in range(PARTICLE_COUNT):
    p = Particle(map)
    p.initialize()
    particles.append(p)

def rotate_particles(angle):
    global particles
    for p in particles:
        if not p.is_alive():
            continue
        p.rotate(angle)

def move_particles(distance):
    global particles
    for p in particles:
        if not p.is_alive():
            continue
        result = p.move(distance)
        if result == False:
            p.initialize()

while not rospy.is_shutdown():
    if robot_state == RobotDecisionState.initial:
        time.sleep(0.1)
        robot_state = RobotDecisionState.pre_thinking

    elif robot_state == RobotDecisionState.pre_thinking:
        # time.sleep(5)

        if random.random() < 0.5:
            # Random rotate
            robot_state = RobotDecisionState.rotating

            angles_deg = [90, -90, 180]
            angle_deg = random.choice(angles_deg)
            print("Rotate " + str(angle_deg) + "deg")
            rotation_angle = angle_deg * PI / 180
            rotate_particles(rotation_angle)
            continue


        sensor_min_val = sensor_range
        if EXTRA_ANGLE_CHECK and sensor_range > 1.5 * translate_distance:
            command_initial_position = robot_position.copy()
            robot_state = RobotDecisionState.rotating
            rotation_angle = -EXTRA_ANGLE_CHECK_ANGLE_DEG * PI / 180
            state_after_stop = RobotDecisionState.making_sure_front_is_accessible_1
            rotate_particles(rotation_angle)
        else:
            robot_state = RobotDecisionState.thinking


    elif robot_state == RobotDecisionState.rotating:

        sensor_min_val = min(sensor_min_val, sensor_range)

        rotation_final_angle = command_initial_position.theta + rotation_angle
        theta_error = normalize_angle(rotation_final_angle - robot_position.theta)

        if abs(theta_error) < ROTATION_ERROR_TOLERANCE:
            robot_state = RobotDecisionState.stopping
            continue

        vel_msg = get_stop_vel_msg()
        vel_msg.angular.z = sign(theta_error) * min(ROTATION_SPEED_MAX, abs(theta_error) * ROTATION_ERROR_TO_VELOCITY_COEF)
        velocity_publisher.publish(vel_msg)

    elif robot_state == RobotDecisionState.translating:
        translation_error = translate_distance - math.sqrt(
            math.pow(robot_position.x - command_initial_position.x, 2) + 
            math.pow(robot_position.y - command_initial_position.y, 2))

        if abs(translation_error) <= TRANSLATION_ERROR_TOLERANCE:
            robot_state = RobotDecisionState.stopping
            continue

        vel_msg = get_stop_vel_msg()
        vel_msg.linear.x = min(TRANSLATION_SPEED_MAX, translation_error * TRANSLATION_ERROR_TO_VELOCITY_COEF)
        velocity_publisher.publish(vel_msg)

    elif robot_state == RobotDecisionState.stopping:
        time.sleep(0.05)

        vel_msg = get_stop_vel_msg()
        velocity_publisher.publish(vel_msg)
        robot_state = state_after_stop

        print("Stop")

    elif robot_state == RobotDecisionState.making_sure_front_is_accessible_1:
        command_initial_position = robot_position.copy()
        robot_state = RobotDecisionState.rotating
        rotation_angle = 2 * EXTRA_ANGLE_CHECK_ANGLE_DEG * PI / 180
        state_after_stop = RobotDecisionState.making_sure_front_is_accessible_2
        rotate_particles(rotation_angle)

    elif robot_state == RobotDecisionState.making_sure_front_is_accessible_2:
        command_initial_position = robot_position.copy()
        robot_state = RobotDecisionState.rotating
        rotation_angle = -EXTRA_ANGLE_CHECK_ANGLE_DEG * PI / 180
        state_after_stop = RobotDecisionState.thinking
        rotate_particles(rotation_angle)

    elif robot_state == RobotDecisionState.thinking:
        print("Sensor min val:" + str(sensor_min_val))

        state_after_stop = RobotDecisionState.pre_thinking
        
        command_initial_position = robot_position.copy()
        if sensor_min_val > (translate_distance * 1.5):
            robot_state = RobotDecisionState.translating
            print("Move forward")
            move_particles(translate_distance)
        else:
            robot_state = RobotDecisionState.rotating

            angles_deg = [90, -90, 180]
            angle_deg = random.choice(angles_deg)
            print("Rotate " + str(angle_deg) + "deg")
            rotation_angle = angle_deg * PI / 180
