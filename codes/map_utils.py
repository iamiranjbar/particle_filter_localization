import math
from shapely.geometry import LineString


def find_intersection(point_1, point_2, point_3, point_4):

    line1 = LineString([tuple(point_1), tuple(point_2)])
    line2 = LineString([tuple(point_3), tuple(point_4)])

    intersection = line1.intersection(line2)
    if intersection:
        point_of_intersection = intersection.x, intersection.y
        return True, point_of_intersection
    else:
        return False, None

def out_of_range(point, offset, boundary):
    first_offset_point = point[0] - offset[0]
    second_offset_point = point[1] - offset[1]
    
    if first_offset_point > boundary[1] or first_offset_point < boundary[0]:
        return True
    elif second_offset_point > boundary[3] or second_offset_point < boundary[2]:
        return True
    else:
        return False


def calculate_distance(first, second):
    return math.sqrt((first[0]- second[0])**2 + (first[1]- second[1])**2)
