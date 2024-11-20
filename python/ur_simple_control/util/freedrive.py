import pinocchio as pin
import numpy as np
import time
import signal
from ur_simple_control.managers import RobotManager
# TODO: put sane default arguments needed just for managers 
# into a separate file, call it default arguments.
# ideally you also only need to add your additional ones
# to this list.
from ur_simple_control.clik.clik_point_to_point import get_args


def handler(signum, frame):
    robot.rtde_control.endFreedriveMode()
    print("done with freedrive, cya")
    exit()

def freedrive(robot):
    robot.rtde_control.freedriveMode()

    while True:
        q = robot.getQ()
        pin.forwardKinematics(robot.model, robot.data, np.array(q))
        print(robot.data.oMi[6])
        print("pin:", *robot.data.oMi[6].translation.round(4), \
                *pin.rpy.matrixToRpy(robot.data.oMi[6].rotation).round(4))
        print("ur5:", *np.array(robot.rtde_receive.getActualTCPPose()).round(4))
        print("q:", *np.array(q).round(4))
        time.sleep(0.005)

if __name__ == "__main__":
    args = get_args()
    robot = RobotManager(args)
    signal.signal(signal.SIGINT, handler)
    freedrive(robot)
    handler(None, None)
    # TODO possibly you want to end freedrive here as well.
    # or end everything imaginable in the signal handler 
