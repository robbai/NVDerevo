import math

from RLUtilities.LinearAlgebra import vec3, dot
from RLUtilities.Maneuvers import Drive

from boost import boostGrabbingSpeed
from shooting import canShoot, startShooting
from util import get_closest_pad, cap, distance_2d, velocity_2d


def shadow(agent):
    agent.drive.step(1 / 60)
    agent.controls = agent.drive.controls
    target = shadowTarget(agent)
    agent.drive.target_pos = target
    agent.drive.target_speed = shadowSpeedOld(agent, target)
    if canShoot(agent):
        startShooting(agent)
    # elif agent.info.ball.pos[2] > 500:
    #     startCatching(agent)
    elif agent.boostGrabs:
        agent.step = "Grabbing Boost"
        target = get_closest_pad(agent).pos
        agent.drive = Drive(agent.info.my_car, target, boostGrabbingSpeed(agent, target))


def shadowTarget(agent):
    centerGoal = agent.info.my_goal.center
    ball = agent.info.ball
    goalToBall = ball.pos - centerGoal
    targetVector = vec3(2 / 3 * goalToBall[0], 2 / 3 * goalToBall[1], 0)
    targetLocation = centerGoal + targetVector
    return targetLocation


def shadowSpeed(agent, targetLocation):
    car = agent.info.my_car
    targetLocal = dot(targetLocation - car.pos, car.theta)
    angle_to_target = cap(math.atan2(targetLocal[1], targetLocal[0]), -3, 3)
    distance_to_target = distance_2d(agent.info.my_car.pos, targetLocation)
    if distance_to_target > 2.5 * velocity_2d(agent.info.my_car.vel):
        return 2300
    else:
        return 2300 - (340 * (angle_to_target ** 2))


def shadowSpeedOld(agent, targetLocation):
    car = agent.info.my_car
    targetLocal = dot(targetLocation - car.pos, car.theta)
    angleToTarget = cap(math.atan2(targetLocal[1], targetLocal[0]), -3, 3)
    distanceToTarget = distance_2d(car.pos, targetLocation)
    return 2300 - cap((900 * (angleToTarget ** 2)), 0, 2200) + cap((distanceToTarget - 1000) / 4, 0, 500)
