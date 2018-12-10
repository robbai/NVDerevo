import math

from RLUtilities.LinearAlgebra import dot

from util import get_closest_pad, cap, distance_2d, velocity_2d


def grabBoost(agent):
    agent.drive.step(1 / 60)
    agent.controls = agent.drive.controls
    agent.drive.target_speed = boostGrabbingSpeed(agent, agent.drive.target_pos)
    if agent.info.my_car.boost > 90 or not get_closest_pad(agent).is_active:
        agent.step = "Ballchasing"


def boostGrabbingAvaiable(agent, ball):
    pad = get_closest_pad(agent)
    distance = distance_2d(agent.info.my_car.pos, pad.pos)
    futureInFrontOfBall = distance_2d(ball.pos, agent.info.my_goal.center) < distance_2d(agent.info.my_car.pos,
                                                                                         agent.info.my_goal.center)
    if distance < 1500 and agent.info.my_car.boost < 34 and pad.is_active and not agent.inFrontOfBall and not futureInFrontOfBall:
        return True
    return False


def boostGrabbingSpeed(agent, target_location):
    car = agent.info.my_car
    targetLocal = dot(target_location - car.pos, car.theta)
    angle_to_target = cap(math.atan2(targetLocal[1], targetLocal[0]), -3, 3)
    distance_to_target = distance_2d(agent.info.my_car.pos, target_location)
    if distance_to_target > 2.5 * velocity_2d(agent.info.my_car.vel):
        return 2300
    else:
        return 2300 - (340 * (angle_to_target ** 2))
