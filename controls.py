from RLUtilities.Maneuvers import Drive, AirDodge

from boost import boost_grabbing_speed, grab_boost
from catching import catching
from catching import start_catching
from defending import defending
from dribble import aim
from shooting import shooting
from shooting import start_shooting, can_shoot
from util import get_closest_pad, can_dodge


def controls(agent):
    if agent.step == "Ballchasing":
        ballChase(agent)
    elif agent.step == "Dodge":
        agent.dodge.step(1 / 60)
        agent.controls = agent.dodge.controls
        agent.controls.boost = 0
        if agent.dodge.finished:
            agent.step = "Ballchasing"
    elif agent.step == "Catching":
        catching(agent)
    elif agent.step == "Defending":
        defending(agent)
    elif agent.step == "Shooting":
        shooting(agent)
    elif agent.step == "Grabbing Boost":
        grab_boost(agent)
    elif agent.step == "Dribbling":
        agent.controls = aim(agent)
        if agent.info.ball.pos[2] < 95:
            start_shooting(agent)
    else:
        agent.step = "Ballchasing"


def ballChase(agent):
    if agent.drive.target_speed != 1399:
        agent.drive.target_speed = 1399
    agent.drive.target_pos = agent.info.ball.pos
    agent.drive.step(1 / 60)
    agent.controls = agent.drive.controls
    if can_dodge(agent, agent.info.ball.pos):
        agent.step = "Dodge"
        agent.dodge = AirDodge(agent.info.my_car, 0.1, agent.info.ball.pos)
    if agent.defending:
        agent.step = "Defending"
    elif agent.info.ball.pos[2] > 250:
        start_catching(agent)
    elif can_shoot(agent):
        start_shooting(agent)
    elif agent.boostGrabs:
        agent.step = "Grabbing Boost"
        target = get_closest_pad(agent).pos
        agent.drive = Drive(agent.info.my_car, target, boost_grabbing_speed(agent, target))
