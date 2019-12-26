""""Module that handles the shooting strategy"""
import math

from rlutilities.linear_algebra import normalize, rotation, vec3, vec2, dot, norm
from rlutilities.mechanics import Dodge

from util import cap, distance_2d, sign, line_backline_intersect, get_speed, velocity_forward, get_bounce
from steps import Step
from halfflip import HalfFlip


def start_shooting(agent):
    """"Method that is run the frame I choose the shooting strategy"""
    agent.step = Step.Shooting
    target = shooting_target(agent)
    speed = get_speed(agent, target)
    agent.drive.target = target
    agent.drive.speed = speed


def shooting(agent):
    """"Method that gives the output for the shooting strategy"""
    ball = agent.info.ball
    car = agent.info.my_car
    our_goal = agent.my_goal.center
    target = shooting_target(agent)
    agent.drive.target = target
    distance = distance_2d(car.location, target)
    vf = velocity_forward(car)
    dodge_overshoot = distance < (abs(vf) + 500) * 1.5
    agent.drive.speed = get_speed(agent, target)
    agent.drive.step(agent.info.time_delta)
    agent.controls = agent.drive.controls
    if agent.defending:
        agent.step = Step.Defending
    elif should_dodge(agent):
        agent.step = Step.Dodge
        agent.dodge = Dodge(car)
        agent.dodge.duration = 0.1
        agent.dodge.target = ball.location
    elif agent.ball_bouncing and not (abs(ball.velocity[2]) < 100
              and sign(agent.team) * ball.velocity[1] < 0) and get_bounce(agent) is not None:
        agent.step = Step.Catching
        agent.drive.target = ball.location
        agent.drive.speed = 1399
    elif vf < -900 and (not dodge_overshoot or distance < 600):
        agent.step = Step.HalfFlip
        agent.halfflip = HalfFlip(car)
    elif not dodge_overshoot and car.location[2] < 80 and\
            (agent.drive.speed > abs(vf) + 300 and 1200 < abs(vf) < 2000 and car.boost <= 25):
        # Dodge towards the target for speed
        agent.step = Step.Dodge
        agent.dodge = Dodge(car)
        agent.dodge.duration = 0.1
        agent.dodge.target = target


def shooting_target(agent):
    """"Method that gives the target for the shooting strategy"""
    ball = agent.info.ball
    car = agent.info.my_car
    car_to_ball = ball.location - car.location
    backline_intersect = line_backline_intersect(
        agent.their_goal.center[1], vec2(car.location), vec2(car_to_ball))
    if abs(backline_intersect) < 700:
        goal_to_ball = normalize(car.location - ball.location)
        error = 0
    else:
        # Right of the ball
        if -500 > backline_intersect:
            target = agent.their_goal.corners[3] + vec3(400, 0, 0)
        # Left of the ball
        elif backline_intersect > 500:
            target = agent.their_goal.corners[2] - vec3(400, 0, 0)
        goal_to_ball = normalize(ball.location - target)
        # Subtract the goal to car vector
        difference = goal_to_ball - normalize(car.location - target)
        error = cap(abs(difference[0]) + abs(difference[1]), 0, 5)

    goal_to_ball_2d = vec2(goal_to_ball[0], goal_to_ball[1])
    test_vector_2d = dot(rotation(0.5 * math.pi), goal_to_ball_2d)
    test_vector = vec3(test_vector_2d[0], test_vector_2d[1], 0)

    distance = cap((40 + distance_2d(ball.location, car.location) * (error ** 2)) / 1.8, 0, 4000)
    location = ball.location + vec3((goal_to_ball[0] * distance), goal_to_ball[1] * distance, 0)

    # this adjusts the target based on the ball velocity perpendicular
    # to the direction we're trying to hit it
    multiplier = cap(distance_2d(car.location, location) / 1500, 0, 2)
    distance_modifier = cap(dot(test_vector, ball.velocity) * multiplier, -1000, 1000)
    location += vec3(
        test_vector[0] * distance_modifier, test_vector[1] * distance_modifier, 0)

    # another target adjustment that applies if the ball is close to the wall
    extra = 3850 - abs(location[0])
    if extra < 0:
        location[0] = cap(location[0], -3850, 3850)
        location[1] = location[1] + (-sign(agent.team) * cap(extra, -800, 800))
    return location


def should_dodge(agent):
    return norm(agent.info.ball.location - agent.info.my_car.location) < 300 and max(dot(agent.info.my_car.velocity, agent.info.ball.location - agent.info.my_car.location), dot(agent.info.my_car.forward(), agent.info.ball.location - agent.info.my_car.location)) > 1300 # Go for it!
    """"Method that checks if we should dodge"""
    car = agent.info.my_car
    their_goal = agent.their_goal
    close_to_goal = distance_2d(car.location, their_goal.center) < 4000
    aiming_for_goal = abs(line_backline_intersect(
        their_goal.center[1], vec2(car.location), vec2(car.forward()))) < 850
    bot_to_target = agent.info.ball.location - car.location
    local_bot_to_target = dot(bot_to_target, agent.info.my_car.rotation)
    angle_front_to_target = math.atan2(local_bot_to_target[1], local_bot_to_target[0])
    close_to_ball = norm(vec2(bot_to_target)) < 750
    good_angle = abs(angle_front_to_target) < math.radians(15)
    return close_to_ball and close_to_goal and aiming_for_goal and good_angle
