from math import cos, atan2, pi, radians, copysign

from rlbot.agents.base_agent import SimpleControllerState
from rlutilities.linear_algebra import vec3, dot, norm, xy, look_at
from rlutilities.mechanics import Drive as RLUDrive, AerialTurn

from util import sign, cap


class CustomDrive:

    def __init__(self, car):
        self.car = car
        self.target = vec3(0, 0, 0)
        self.speed = 2300
        self.controls = SimpleControllerState()
        self.finished = False
        self.rlu_drive = RLUDrive(self.car)
        self.update_rlu_drive()
        self.power_turn = True  # Handbrake while reversing to turn around quickly
        self.aerial_turn = AerialTurn(car)
        self.kickoff = False

    def step(self, dt: float):
        self.speed = abs(self.speed)
        car_to_target = (self.target - self.car.location)
        local_target = dot(car_to_target, self.car.rotation)
        angle = atan2(local_target[1], local_target[0])
        vel = norm(self.car.velocity)
        in_air = (not self.car.on_ground)
        on_wall = (self.car.location[2] > 250 and not in_air)

        reverse = (cos(angle) < 0 and not (on_wall or in_air or self.kickoff))

        get_off_wall = (on_wall and local_target[2] > 450)
        if get_off_wall:
            car_to_target[2] = -self.car.location[2]
            local_target = dot(car_to_target, self.car.rotation)
            angle = atan2(local_target[1], local_target[0])

        print(self.kickoff)
        max_speed = self.determine_max_speed(local_target)

        self.update_rlu_drive(reverse, max_speed)
        self.rlu_drive.step(dt)
        self.finished = self.rlu_drive.finished

        self.controls = self.rlu_drive.controls
        self.controls.handbrake = False
       
        if reverse:
            angle = -invert_angle(angle)
            if self.power_turn and not on_wall:
                angle *= -1
                self.controls.handbrake = (vel > 200)
            self.controls.steer = cap(angle * 3, -1, 1)
            self.controls.boost = False
        if not self.controls.handbrake:
            self.controls.handbrake = (abs(angle) > radians(70) and vel > 500 and not on_wall)
        if self.controls.handbrake:
            self.controls.handbrake = (dot(self.car.velocity, car_to_target) > -150)

        if in_air:
            self.aerial_turn.target = look_at(xy(car_to_target), vec3(0, 0, 1))
            self.aerial_turn.step(dt)
            aerial_turn_controls = self.aerial_turn.controls
            self.controls.pitch = aerial_turn_controls.pitch
            self.controls.yaw = aerial_turn_controls.yaw
            self.controls.roll = aerial_turn_controls.roll
            self.controls.boost = False

    def update_rlu_drive(self, reverse: bool = False, max_speed: float = 2200):
        self.target = self.target
        self.rlu_drive.target = self.target
        self.rlu_drive.speed = cap(self.speed * (-1 if reverse else 1), -max_speed, max_speed)

    def determine_max_speed(self, local_target):
        low = 100
        high = 2200
        if self.kickoff:
            return high
        for i in range(5):
            mid = (low + high) / 2
            radius = (1 / RLUDrive.max_turning_curvature(mid))
            local_circle = vec3(0, copysign(radius, local_target[1]), 0)
            dist = norm(local_circle - xy(local_target))
            if dist < radius:
                high = mid
            else:
                low = mid
        return high


def invert_angle(angle: float):
    return angle - sign(angle) * pi
