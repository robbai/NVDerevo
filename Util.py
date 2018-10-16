import math, time, random
from rlbot.utils.game_state_util import GameState, BallState, CarState, Physics, Vector3 as StateVector3, Rotator
from rlbot.agents.base_agent import BaseAgent, SimpleControllerState

GOAL_WIDTH = 1784
FIELD_LENGTH = 10240
FIELD_WIDTH = 8192

def abc(a,b,c):
    inside = (b**2) - (4*a*c)
    if inside < 0 or a == 0:
        return 0.1
    else:
        n = ((-b - math.sqrt(inside))/(2*a))
        p = ((-b + math.sqrt(inside))/(2*a))
        if p > n:
            return p
        return n

def getClosestPad(agent):
    pads = agent.info.boost_pads
    closestPad = None
    distToClosestPad = math.inf
    for i in range(len(pads)):
        if(distance2D(agent.deevo, pads[i]) < distToClosestPad):
            distToClosestPad = distance2D(agent.deevo, pads[i])
            closestPad = pads[i]
    return closestPad

def angle2D(targetLocation, objectLocation):
    difference = targetLocation - objectLocation
    return math.atan2(difference[1], difference[0])

def timeZ(ball):
    rate = 0.97
    return abc(-325, ball.vel[2] * rate, ball.pos[2]-92.75)

def ballReady(agent):
    ball = agent.info.ball
    if abs(ball.vel[2]) < 150 and timeZ(ball) < 1:
            return True
    return False

def ballProject(agent):
    goal = agent.info.their_goal
    goalToBall = normalize(agent.ball.location - goal)
    diff = agent.info.my_car.pos - agent.info.my_car
    return diff * goalToBall

def dpp(targetLocation,targetSpeed,location,velocity):
    d = distance2D(targetLocation,location)
    if d != 0:
        return (((targetLocation[0] - location[0]) * (targetSpeed[0] - velocity[0])) + ((targetLocation[1] - location[1]) * (targetSpeed[1] - velocity[1])))/d
    else:
        return 0

def cap(x, low, high):
    if x < low:
        return low
    elif x > high:
        return high
    else:
        return x

def distance2D(targetObject, ourObject):
    difference = targetObject - ourObject
    return math.sqrt(difference[0]**2 + difference[1]**2)

def boost_needed(self, initialSpeed, targetSpeed):
    p1 = 6.31e-06
    p2 = 0.010383
    p3 = 1.3183
    initialBoost = p1*initialSpeed**2 + p2*initialSpeed + p3
    targetBoost = p1*targetSpeed**2 + p2*targetSpeed + p3
    boostNeeded = targetBoost - initialBoost
    return boostNeeded

def renderString(self, string):
        self.renderer.begin_rendering()
        self.renderer.draw_string_2d(20, 20, 3, 3, string, self.renderer.red())
        self.renderer.end_rendering()

def convert_input(original):
    converted = SimpleControllerState()
    converted.steer = original.steer
    converted.throttle = original.throttle
    converted.pitch = original.pitch
    converted.yaw = original.yaw
    converted.roll = original.roll
    converted.jump = original.jump
    converted.boost = original.boost
    converted.handbrake = original.slide
    return converted

# def setState(self):
#     pitch = random.uniform(0, 1)*2*math.pi
#     yaw = random.uniform(0, 1)*2*math.pi
#     roll = random.uniform(0, 1)*2*math.pi
#     xSpeed = random.uniform(-1, 1)*1000
#     ySpeed = random.uniform(-1, 1)*1000
#     zSpeed = random.uniform(-1, 1)*1000
#     xLocation = random.uniform(-1, 1)*4096
#     yLocation = random.uniform(-1, 1)*5120
#     carState = CarState(jumped=False, double_jumped=False, boost_amount=87, physics=Physics(velocity=StateVector3(xSpeed, ySpeed, zSpeed), location=StateVector3(xLocation, yLocation, 500), rotation=Rotator(pitch, yaw, roll), angular_velocity=StateVector3(1000, 1000, 1000)))
#     carState2 = CarState(physics=Physics(location=StateVector3(10000, 10000, 10000)))
#     ballState = BallState(physics=Physics(velocity=StateVector3(0, 0, 0), location=StateVector3(0, 2500, 0), rotation=Rotator(0, 0, 0), angular_velocity=StateVector3(0, 0, 0)))
#     gameState = GameState(ball=ballState, cars={self.index: carState, 1: carState2})
#     self.halfFlipping = False
#     self.set_game_state(gameState)


def setState(self):
    carState = CarState(jumped=False, double_jumped=False, boost_amount=87, physics=Physics(velocity=StateVector3(0, 0, 0), location=StateVector3(0, 300, 0), rotation=Rotator(0, - math.pi/2, 0), angular_velocity=StateVector3(0, 0, 0)))
    carState2 = CarState(physics=Physics(location=StateVector3(10000, 10000, 10000)))
    ballState = BallState(physics=Physics(velocity=StateVector3(0, 0, 0), location=StateVector3(0, -2500, 0), rotation=Rotator(0, 0, 0), angular_velocity=StateVector3(0, 0, 0)))
    gameState = GameState(ball=ballState, cars={self.index: carState, 0: carState2})
    self.set_game_state(gameState)