import math

class RobotDecisionState:
    initial = 0
    translating = 1
    rotating = 2
    pre_thinking = 3
    thinking = 4
    stopping = 5
    making_sure_front_is_accessible_1 = 10
    making_sure_front_is_accessible_2 = 11
    halt = 999

class RobotWorldState:
    def __init__(self, x=0, y=0, theta=0):
        self.x = x
        self.y = y
        self.theta = theta

    def copy(self):
        return RobotWorldState(x=self.x, y=self.y, theta=self.theta)
       