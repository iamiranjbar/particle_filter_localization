from codes.visualization import get_sensor_line, draw_status, draw_sensor_line
from codes.map_utils import find_intersection, calculate_distance
from multiprocessing import Pool
import scipy.stats as stats
import numpy as np
import math

def generate_particles(map, new_particles_count):
    min_x, max_x, min_y, max_y = map.boundary()
    new_particles = np.empty((new_particles_count, 3))
    new_particles[:, 0] = np.random.uniform(min_x, max_x, size=new_particles_count) + map.global_map_poses[0]
    new_particles[:, 1] = np.random.uniform(min_x, max_x, size=new_particles_count) + map.global_map_poses[1]
    new_particles[:, 2] = np.random.choice([-90, 90, 180, 0], size=new_particles_count) * math.pi / 180.0
    return new_particles

def rotate_particles(particles, angle):
    for i in range(len(particles)):
        particles[i][2] += angle
        # TODO: Add normal noise model
        while particles[i][2] > math.pi:
            particles[i][2] -= 2 * math.pi
        while particles[i][2] < -math.pi:
            particles[i][2] += 2 * math.pi
    return particles

def move_particles(particles, distance):
    for i in range(len(particles)):
        dx = distance * math.cos(particles[i][2])
        dy = distance * math.sin(particles[i][2])

        # TODO: Add normal noise model

        particles[i][0] += dx
        particles[i][1] += dy
    return particles

map_lines = []
def get_particle_weight(input):
    global map_lines

    particle, has_collision, sensor_range = input

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

    weight = stats.norm(min_distance, 0.01049).pdf(sensor_range)

    return weight

def resample_particles(particles, map, sensor_range):
    global map_lines

    map_lines = map.get_lines()
    weights = []
    multiprocessing_list = []
    for i in range(len(particles)):
        has_collision = map.is_invalid_point(particles[i])
        multiprocessing_list.append((particles[i], has_collision, sensor_range))
    
    print("Starting multiprocessing")
    pool = Pool(6)
    import traceback
    try:
        weights = np.array(pool.map(get_particle_weight, multiprocessing_list))
    except:
        traceback.print_exc()
    pool.close()
    
    print("Finished multiprocessing")
    prob_sum = np.sum(weights)

    weights /= prob_sum

    indexes = np.random.choice(len(particles), int(0.8 * len(particles)), p=weights)
    resampled_particles = np.array([particles[i] for i in indexes])

    # add random particles
    new_particles = generate_particles(map, int(0.2 * len(particles)))

    return np.concatenate([resampled_particles, new_particles])
