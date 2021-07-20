import math
import matplotlib.pyplot as plt


def draw_sensor_line(particle, sensor_range=0.4):
    particle_x = particle[0]
    particle_y = particle[1]
    particle_tetha = particle[2]
    laser_end_x = sensor_range * math.cos(particle_tetha) + particle_x
    laser_end_y = sensor_range * math.sin(particle_tetha) + particle_y
    laser_start_point = [particle_x, particle_y]
    laser_end_point = [laser_end_x, laser_end_y]
    sensor_line = [laser_start_point, laser_end_point]
    plt.plot([sensor_line[0][1], sensor_line[1][1]] , [sensor_line[0][0], sensor_line[1][0]])


def draw_status(point, color):
    x = point[0]
    y = point[1]
    tetha = point[2]
    delta_x = 0.00001 * math.cos(tetha)
    delta_y = 0.00001 * math.sin(tetha)
    plt.arrow(x, y, delta_x, delta_y , head_width = 0.02, fill=False, overhang = 0.6, color=color)


def draw_dot_line(start_point, end_point, color):
    start_x, start_y = start_point[0], start_point[1]
    end_x, end_y = end_point[0], end_point[1]
    plt.plot([start_x, end_x], [start_y, end_y], color=color, linestyle=':')
