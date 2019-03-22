from rlutilities.linear_algebra import *


class Goal:
    WIDTH = 1784.0
    HEIGHT = 640.0
    DISTANCE = 5120.0

    def __init__(self, team, fieldInfo=None):

        self.team = team
        self.sign = -1 if team == 0 else 1
        for i in range(len(fieldInfo.goals)):
            current = fieldInfo.goals[i]
            current_pos = current.location
            if current.team_num == team:
                self.center = vec3(current_pos.x, current_pos.y, current_pos.z)
                if abs(self.center[1]) > 5000:
                    self.corners = [
                        vec3(current_pos.x - Goal.WIDTH / 2.0, current_pos.y, current_pos.z - Goal.HEIGHT / 2.0),
                        vec3(current_pos.x + Goal.WIDTH / 2.0, current_pos.y, current_pos.z - Goal.HEIGHT / 2.0),
                        vec3(current_pos.x + Goal.WIDTH / 2.0, current_pos.y, current_pos.z + Goal.HEIGHT / 2.0),
                        vec3(current_pos.x - Goal.WIDTH / 2.0, current_pos.y, current_pos.z + Goal.HEIGHT / 2.0)
                    ]
                else:
                    radius = 3564 - abs(current_pos.y)
                    self.corners = [
                        vec3(-radius, current_pos.y - radius, current_pos.z * 2.0),
                        vec3(radius, current_pos.y - radius, current_pos.z * 2.0),
                        vec3(radius, current_pos.y + radius, current_pos.z * 2.0),
                        vec3(-radius, current_pos.y + radius, current_pos.z * 2.0)
                    ]
                break

    def inside(self, p):
        if self.team == 0:
            return p[1] < -Goal.DISTANCE
        else:
            return p[1] > Goal.DISTANCE