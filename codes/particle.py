from .model import RobotWorldState
import random
import math

class Particle:
    def __init__(self, _map):
        self.map = _map
        self.alive = False
        self.weight = 0

    def copy(self):
        p = Particle(self.map)
        p.alive = self.alive
        p.state = self.state
        return p

    def initialize(self):
        min_x, max_x, min_y, max_y = self.map.boundary()

        self.state = None
        while self.map.is_invalid_point(self.state):
            x = min_x + random.random() * (max_x - min_x)
            y = min_y + random.random() * (max_y - min_y)
            theta = random.choice([0, 45, 90, 135, 180, -45, -90, -135]) * math.pi / 180
            self.state = RobotWorldState(x, y, theta)
        
        self.alive = True

    def get_state_list(self):
        return [self.state.x, self.state.y, self.state.theta]

    def rotate(self, theta):
        self.state.theta += theta
        # TODO: Add random noise based on rotate model

        while self.state.theta > math.pi:
            self.state.theta -= 2 * math.pi
        while self.state.theta < -math.pi:
            self.state.theta += 2 * math.pi
    
    def move(self, distance):
        dx = distance * math.cos(self.state.theta)
        dy = distance * math.sin(self.state.theta)

        # TODO: Add random noise based on translate model

        # if self.map.is_invalid_path(self.state, dx, dy):
        #     self.alive = False

        self.state.x += dx
        self.state.y += dy

        if self.map.is_invalid_point(self.state):
            self.alive = False

        return self.alive

    def is_alive(self):
        return self.alive

