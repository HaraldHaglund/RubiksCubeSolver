import numpy as np
import time
import argparse
from functools import partial
from ur_simple_control.managers import ControlLoopManager, RobotManager
# TODO merge the 2 clik files
from ur_simple_control.clik.clik_point_to_point import getClikController, controlLoopClik, moveL, compliantMoveL
# TODO write this in managers and automate name generation
from ur_simple_control.util.logging_utils import saveLog


"""
Every imaginable magic number, flag and setting is put in here.
This way the rest of the code is clean.
If you want to know what these various arguments do, just grep 
though the code to find them (but replace '-' with '_' in multi-word arguments).
All the sane defaults are here, and you can change any number of them as an argument
when running your program. If you want to change a drastic amount of them, and
not have to run a super long commands, just copy-paste this function and put your
own parameters as default ones.
NOTE1: the args object obtained from args = parser.get_args()
is pushed all around the rest of the code (because it's tidy), so don't change their names.
NOTE2: that you need to copy-paste and add aguments you need
to the script you will be writing. This is kept here 
only as a convenient reference point.
"""
def get_args():
    parser = argparse.ArgumentParser(description='Run closed loop inverse kinematics \
            of various kinds. Make sure you know what the goal is before you run!',
            formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--simulation', action=argparse.BooleanOptionalAction, 
            help="whether you are running the UR simulator", default=False)
    parser.add_argument('--robot-ip', type=str, 
            help="robot's ip address (only needed if running on the real robot)", \
                    default="192.168.1.102")
    parser.add_argument('--pinocchio-only', action=argparse.BooleanOptionalAction, 
            help="whether you want to just integrate with pinocchio", default=False)
    parser.add_argument('--visualize-manipulator', action=argparse.BooleanOptionalAction, 
            help="whether you want to visualize the manipulator and workspace with meshcat", default=False)
    parser.add_argument('--real-time-plotting', action=argparse.BooleanOptionalAction, 
            help="whether you want to have some real-time matplotlib graphs (parts of log_dict you select)", default=False)
    parser.add_argument('--gripper', type=str, \
            help="gripper you're using (no gripper is the default)", 
                        default="none", choices=['none', 'robotiq', 'onrobot'])
    parser.add_argument('--goal-error', type=float, \
            help="the final position error you are happy with", default=1e-2)
    parser.add_argument('--max-iterations', type=int, \
            help="maximum allowable iteration number (it runs at 500Hz)", default=100000)
    parser.add_argument('--acceleration', type=float, \
            help="robot's joints acceleration. scalar positive constant, max 1.7, and default 0.4. \
                   BE CAREFUL WITH THIS. the urscript doc says this is 'lead axis acceleration'.\
                   TODO: check what this means", default=0.3)
    parser.add_argument('--speed-slider', type=float,\
            help="cap robot's speed with the speed slider \
                    to something between 0 and 1, 0.5 by default \
                    BE CAREFUL WITH THIS.", default=0.5)
    parser.add_argument("--start-from-current-pose", action=argparse.BooleanOptionalAction, \
            help="if connected to the robot, read the current pose and set it as the initial pose for the robot. \
                 very useful and convenient when running simulation before running on real", \
                         default=False)
    parser.add_argument('--tikhonov-damp', type=float, \
            help="damping scalar in tikhonov regularization", default=1e-3)
    parser.add_argument('--minimum-detectable-force-norm', type=float, \
            help="we need to disregard noise to converge despite filtering. \
                  a quick fix is to zero all forces of norm below this argument threshold.",
                 default=3.0)
    # TODO add the rest
    parser.add_argument('--clik-controller', type=str, \
            help="select which click algorithm you want", \
            default='dampedPseudoinverse', choices=['dampedPseudoinverse', 'jacobianTranspose', 'invKinmQP'])
        # maybe you want to scale the control signal
    parser.add_argument('--controller-speed-scaling', type=float, \
            default='1.0', help='not actually_used atm')
    parser.add_argument('--debug-prints', action=argparse.BooleanOptionalAction, \
            help="print some debug info", default=False)
    parser.add_argument('--save-log', action=argparse.BooleanOptionalAction, \
            help="save log data folder in whatever directory you called this in. if it doesn't exists there, it's saved to /tmp/data", default=False)
    parser.add_argument('--alpha', type=float, \
            help="force feedback proportional coefficient", \
            default=0.01)
    parser.add_argument('--beta', type=float, \
            help="low-pass filter beta parameter", \
            default=0.01)
    parser.add_argument('--past-window-size', type=int, \
            help="how many timesteps of past data you want to save", default=5)

    args = parser.parse_args()
    if args.gripper and args.simulation:
        raise NotImplementedError('Did not figure out how to put the gripper in \
                the simulation yet, sorry :/ . You can have only 1 these flags right now')
    return args

if __name__ == "__main__": 
    args = get_args()
    robot = RobotManager(args)
    Mgoal = robot.defineGoalPointCLI()
    log_dict, final_iteration = compliantMoveL(args, robot, Mgoal)
    robot.closeGripper()
    time.sleep(2.0)
    robot.openGripper()
    if not args.pinocchio_only:
        print("stopping via freedrive lel")
        robot.rtde_control.freedriveMode()
        time.sleep(0.5)
        robot.rtde_control.endFreedriveMode()

    if args.visualize_manipulator:
        robot.killManipulatorVisualizer()
    
    if args.save_log:
        saveLog(log_dict, final_iteration, args)
    #loop_manager.stopHandler(None, None)

