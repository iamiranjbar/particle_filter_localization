import math
import matplotlib.pyplot as plt
import xml.etree.ElementTree as ET
from shapely.geometry import Point, Polygon

class Map:
    def __init__(self, map_address):
        self.rectangles = []
        self.centers = []
        self.global_map_poses = []
        self.parse_tree(map_address)
        self.add_offset()

    def parse_tree(self, map_address):
        tree = ET.parse(map_address)
        root = tree.getroot()
        world = root[0]
        for model in world.iter('model'):
            try:
                for child in model:
                    if child.tag == 'pose':
                        self.extract_pose_tag(child)
                    if child.tag == 'link':
                        self.extract_link_tag(child)                        
            except Exception:
                pass
    
    def extract_pose_tag(self, child):
        global_map_poses_temp = [float(x) for x in child.text.split(' ')]
        # TODO: Why?!
        if global_map_poses_temp[0] != 0:
            self.global_map_poses = global_map_poses_temp

    def extract_link_tag(self, child):
        child_pose = None
        child_geometry = None

        for pose_tag in child.iter('pose'):
            child_pose = pose_tag.text.split(' ')
            break

        for collision in child.iter('collision'):
            child_geometry = collision[3][0][0].text.split(' ')

        point_1 = [float(child_pose[0]) - float(child_geometry[0])*math.cos(float(child_pose[5]))/2
                   -float(child_geometry[1])*math.sin(float(child_pose[5]))/2 ,
                   float(child_pose[1]) - float(child_geometry[0])*math.sin(float(child_pose[5]))/2
                   +float(child_geometry[1])*math.cos(float(child_pose[5]))/2]

        point_2 = [float(child_pose[0]) - float(child_geometry[0])*math.cos(float(child_pose[5]))/2
                   +float(child_geometry[1])*math.sin(float(child_pose[5]))/2 ,
                   float(child_pose[1]) - float(child_geometry[0])*math.sin(float(child_pose[5]))/2
                   -float(child_geometry[1])*math.cos(float(child_pose[5]))/2]

        point_3 = [float(child_pose[0]) + (float(child_geometry[0]) * math.cos(float(child_pose[5])) / 2)
                   + float(child_geometry[1]) * math.sin(float(child_pose[5])) / 2 ,
                   float(child_pose[1]) + float(child_geometry[0]) * math.sin(float(child_pose[5])) / 2
                   -float(child_geometry[1]) * math.cos(float(child_pose[5])) / 2 ]

        point_4 = [float(child_pose[0]) + float(child_geometry[0]) * math.cos(float(child_pose[5])) / 2
                   -float(child_geometry[1])*math.sin(float(child_pose[5]))/2 ,
                   float(child_pose[1]) + float(child_geometry[0])*math.sin(float(child_pose[5]))/2
                   +float(child_geometry[1])*math.cos(float(child_pose[5]))/2]

        self.rectangles.append([point_1 ,point_2 ,point_3, point_4, point_1])
        self.centers.append([child_pose[0],child_pose[1]])

    def plot(self):
        for rectangle in self.rectangles:
            rectangle = list(zip(*rectangle))
            plt.plot(rectangle[1], rectangle[0], c='black')
        #plt.show()


    def get_lines(self):
        lines = []
        for points in self.rectangles:
            lines.append([ points[0] , points[1]] )
            lines.append([ points[1] , points[2]] )
            lines.append([ points[2] , points[3]] )
            lines.append([ points[3] , points[0]] )
        return lines

    def get_polygons(self):
        polygons = []
        for points in self.rectangles:
            polygons.append(Polygon(
                [tuple(points[0]),
                tuple(points[1]),
                tuple(points[2]),
                tuple(points[3])
                ]))
        return polygons

    def add_offset(self):
        offset = self.global_map_poses
        for index, points in enumerate(self.rectangles): 
            self.rectangles[index] = [
                    [points[0][0] + offset[0] , points[0][1] + offset[1]],
                    [points[1][0] + offset[0] , points[1][1] + offset[1]],
                    [points[2][0] + offset[0] , points[2][1] + offset[1]],
                    [points[3][0] + offset[0] , points[3][1] + offset[1]],
                    [points[0][0] + offset[0] , points[0][1] + offset[1]],
                ]

    def is_invalid_point(self, point):
        point = Point(point[0], point[1])
        for rect in self.get_polygons():
            if rect.contains(point):
                return True

        min_x, max_x, min_y, max_y = self.boundary()
        if point.x < min_x or point.x > max_x or point.y < min_y or point.y > max_y:
            return True

        return False

    def boundary(self):
        xs = []
        ys = [] 

        for points in self.rectangles:
            for point in points:
                # not In chapas
                xs.append(float(point[0]))
                ys.append(float(point[1]))

        return min(xs), max(xs), min(ys), max(ys)

if __name__ == "__main__":
    map = Map('./worlds/sample1.world')
    map.plot()
    plt.show()
