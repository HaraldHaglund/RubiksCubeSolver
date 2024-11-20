import numpy as np
import pinocchio as pin
import crocoddyl
from ur_simple_control.managers import ControlLoopManager, RobotManager

def IKOCP(robot : RobotManager, goal : pin.SE3):
    state = crocoddyl.StateMultibody(robot.model)
