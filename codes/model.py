class RobotDecisionState:
    translating = 1
    rotating = 2
    updating = 5
    movement = 7
    halt = 999

    making_sure_front_is_accessible_1 = 10
    making_sure_front_is_accessible_2 = 11

class RobotWorldState:
    def __init__(self, x=0, y=0, theta=0):
        self.x = x
        self.y = y
        self.theta = theta

    def get_state_list(self):
        return [self.x, self.y, self.theta]

    def copy(self):
        return RobotWorldState(x=self.x, y=self.y, theta=self.theta)
       