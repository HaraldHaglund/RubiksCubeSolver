# this is a quickest possible solution,
# not a good one
# please change at the earliest convenience

import pinocchio as pin
import numpy as np
import copy
import argparse
from functools import partial
from ur_simple_control.managers import ControlLoopManager, RobotManager
import time

def moveJControlLoop(q_desired, robot, i, past_data):
    """
    moveJControlLoop
    ---------------
    most basic P control for joint space point-to-point motion, actual control loop.
    """
    breakFlag = False
    save_past_dict = {}
    # you don't even need forward kinematics for this lmao
    q = robot.getQ()
    # TODO: be more intelligent with qs
    q = q[:6]
    q_desired = q_desired[:6]
    q_error = q_desired - q

    # STOP MUCH BEFORE YOU NEED TO FOR THE DEMO
    # EVEN THIS MIGHT BE TOO MUCH
    # TODO fix later obviously
    if np.linalg.norm(q_error) < 1e-3:
        breakFlag = True
    # stupid hack, for the love of god remove this
    # but it should be small enough lel
    # there. fixed. tko radi taj i grijesi, al jebemu zivot sta je to bilo
    K = 120
    #print(q_error)
    # TODO: you should clip this
    qd = K * q_error * robot.dt
    #qd = np.clip(qd, robot.acceleration, robot.acceleration)
    robot.sendQd(qd)
    return breakFlag, {}, {}

# TODO:
# fix this by tuning or whatever else.
# MOVEL works just fine, so apply whatever's missing for there here
# and that's it.
def moveJ(args, robot, q_desired):
    """
    moveJ
    ---------------
    most basic P control for joint space point-to-point motion.
    just starts the control loop without any logging.
    """
    assert type(q_desired) == np.ndarray
    controlLoop = partial(moveJControlLoop, q_desired, robot)
    # we're not using any past data or logging, hence the empty arguments
    loop_manager = ControlLoopManager(robot, controlLoop, args, {}, {})
    log_dict, final_iteration = loop_manager.run()
    # TODO: remove, this isn't doing anything
    #time.sleep(0.01)
    if args.debug_prints:
        print("MoveJ done: convergence achieved, reached destionation!")


def jointTrajFollowingPIDControlLoop():
    pass


def jointTrajFollowingPID():
    pass
