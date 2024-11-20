
import pinocchio as pin
import numpy as np
import matplotlib.pyplot as plt
import copy
import argparse
import time
from functools import partial
from ur_simple_control.visualize.visualize import plotFromDict
from ur_simple_control.util.draw_path import drawPath
from ur_simple_control.dmp.dmp import DMP, NoTC,TCVelAccConstrained 
# TODO merge these clik files as well, they don't deserve to be separate
# TODO but first you need to clean up clik.py as specified there
from ur_simple_control.clik.clik_point_to_point import getClikController, moveL, moveUntilContact
from ur_simple_control.clik.clik_trajectory_following import map2DPathTo3DPlane, clikCartesianPathIntoJointPath
from ur_simple_control.managers import ControlLoopManager, RobotManager
from ur_simple_control.util.calib_board_hacks import calibratePlane, getSpeedInDirectionOfN
from ur_simple_control.basics.basics import moveJ
import matplotlib

#######################################################################
#                            arguments                                #
#######################################################################

def getArgs():
    #######################################################################
    #                          generic arguments                          #
    #######################################################################
    parser = argparse.ArgumentParser(description='Make a drawing on screen,\
            watch the robot do it on the whiteboard.',
            formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    # TODO this one won't really work but let's leave it here for the future
    parser.add_argument('--simulation', action=argparse.BooleanOptionalAction, 
            help="whether you are running the UR simulator. \
                    NOTE: doesn't actually work because it's not a physics simulator", \
                    default=False)
    parser.add_argument('--pinocchio-only', action=argparse.BooleanOptionalAction, 
            help="whether you want to just integrate with pinocchio.\
                    NOTE: doesn't actually work because it's not a physics simulator", \
                    default=False)
    parser.add_argument('--visualize-manipulator', action=argparse.BooleanOptionalAction, 
            help="whether you want to visualize the manipulator and workspace with meshcat", default=False)
    parser.add_argument('--real-time-plotting', action=argparse.BooleanOptionalAction, 
            help="whether you want to have some real-time matplotlib graphs (parts of log_dict you select)", default=False)
    parser.add_argument('--gripper', action=argparse.BooleanOptionalAction, \
            help="whether you're using the gripper", default=False)
    parser.add_argument('--acceleration', type=float, \
            help="robot's joints acceleration. scalar positive constant, \
            max 1.7, and default 0.4. \
            BE CAREFUL WITH THIS. the urscript doc says this is 'lead axis acceleration'.\
            TODO: check what this means", default=0.3)
    parser.add_argument('--speed-slider', type=float,\
            help="cap robot's speed with the speed slider \
                    to something between 0 and 1, 1.0 by default because for dmp. \
                    BE CAREFUL WITH THIS.", default=1.0)
    parser.add_argument('--max-iterations', type=int, \
            help="maximum allowable iteration number (it runs at 500Hz)", default=500000)
    #######################################################################
    #                 your controller specific arguments                  #
    #######################################################################
    # not applicable here, but leaving it in the case it becomes applicable
    # it's also in the robot manager even though it shouldn't be
    parser.add_argument('--past-window-size', type=int, \
            help="how many timesteps of past data you want to save", default=5)
    parser.add_argument('--goal-error', type=float, \
            help="the final position error you are happy with. NOTE: not used here", \
            default=1e-3)
    # TODO: test the interaction of this and the overall demo
    parser.add_argument('--tikhonov-damp', type=float, \
            help="damping scalar in tikhonov regularization.\
            This is used when generating the joint trajectory from the drawing.", \
            default=1e-2)
    # TODO add the rest
    parser.add_argument('--clik-controller', type=str, \
            help="select which click algorithm you want", \
            default='dampedPseudoinverse', \
            choices=['dampedPseudoinverse', 'jacobianTranspose'])
        # maybe you want to scale the control signal
    parser.add_argument('--controller-speed-scaling', type=float, \
            default='1.0', help='not actually_used atm')
    #############################
    #  dmp  specific arguments  #
    #############################
    parser.add_argument('--temporal-coupling', action=argparse.BooleanOptionalAction, \
            help="whether you want to use temporal coupling", default=True)
    parser.add_argument('--kp', type=float, \
            help="proportial control constant for position errors", \
            default=1.0)
    parser.add_argument('--kv', type=float, \
            help="damping in impedance control", \
            default=0.001)
    parser.add_argument('--tau0', type=float, \
            help="total time needed for trajectory. if you use temporal coupling,\
                  you can still follow the path even if it's too fast", \
            default=5)
    parser.add_argument('--gamma-nominal', type=float, \
            help="positive constant for tuning temporal coupling: the higher,\
            the fast the return rate to nominal tau", \
            default=1.0)
    parser.add_argument('--gamma-a', type=float, \
            help="positive constant for tuning temporal coupling, potential term", \
            default=0.5)
    parser.add_argument('--eps-tc', type=float, \
            help="temporal coupling term, should be small", \
            default=0.001)
    parser.add_argument('--alpha', type=float, \
            help="force feedback proportional coefficient", \
            default=0.05)
    parser.add_argument('--beta', type=float, \
            help="low-pass filter beta parameter", \
            default=0.01)
    #######################################################################
    #                       task specific arguments                       #
    #######################################################################
    # TODO measure this for the new board
    parser.add_argument('--board-width', type=float, \
            help="width of the board (in meters) the robot will write on", \
            default=0.3)
    parser.add_argument('--board-height', type=float, \
            help="height of the board (in meters) the robot will write on", \
            default=0.3)
    parser.add_argument('--calibration', action=argparse.BooleanOptionalAction, \
            help="whether you want to do calibration", default=False)
    parser.add_argument('--draw-new', action=argparse.BooleanOptionalAction, \
            help="whether draw a new picture, or use the saved path path_in_pixels.csv", default=True)
    parser.add_argument('--pick_up_marker', action=argparse.BooleanOptionalAction, \
            help="""
    whether the robot should pick up the marker.
    NOTE: THIS IS FROM A PREDEFINED LOCATION.
    """, default=True)
    parser.add_argument('--find-marker-offset', action=argparse.BooleanOptionalAction, \
            help="""
    whether you want to do find marker offset (recalculate TCP
    based on the marker""", default=False)
    parser.add_argument('--n-calibration-tests', type=int, \
            help="number of calibration tests you want to run", default=10)
    parser.add_argument('--clik-goal-error', type=float, \
            help="the clik error you are happy with", default=1e-2)
    parser.add_argument('--max-init-clik-iterations', type=int, \
            help="number of max clik iterations to get to the first point", default=10000)
    parser.add_argument('--max-running-clik-iterations', type=int, \
            help="number of max clik iterations between path points", default=1000)
    parser.add_argument('--debug-prints', action=argparse.BooleanOptionalAction, \
            help="print some debug info", default=False)

    args = parser.parse_args()
    if args.gripper and args.simulation:
        raise NotImplementedError('Did not figure out how to put the gripper in \
                the simulation yet, sorry :/ . You can have only 1 these flags right now')
    return args


if __name__ == "__main__":
    #######################################################################
    #                           software setup                            #
    #######################################################################

    #matplotlib.use('tkagg')
    args = getArgs()
    if args.debug_prints:
        print("you will get a lot of stuff printed out, as requested")
    clikController = getClikController(args)
    robot = RobotManager(args)

    # calibrate FT first
    #wrench_offset = robot.calibrateFT()
    
    # TODO and NOTE the weight, TCP and inertial matrix needs to be set on the robot
    # you already found an API in rtde_control for this, just put it in initialization 
    # under using/not-using gripper parameters
    # ALSO NOTE: to use this you need to change the version inclusions in
    # ur_rtde due to a bug there in the current ur_rtde + robot firmware version 
    # (the bug is it works with the firmware verion, but ur_rtde thinks it doesn't)
    # here you give what you're saving in the rolling past window 
    # it's initial value.
    # controlLoopManager will populate the queue with these initial values
#    save_past_dict = {
#            'wrench' : np.zeros(6),
#        }
#    # here you give it it's initial value
#    log_item = {
#            'qs' : np.zeros(robot.n_arm_joints),
#            'dqs' : np.zeros(robot.n_arm_joints),
#            'wrench_raw' : np.zeros(6),
#            'wrench_used' : np.zeros(6),
#        }
    q_init = robot.getQ()

#    controlLoop = partial(controlLoopPointImpedance, q_init, controller, robot)
#    loop_manager = ControlLoopManager(robot, controlLoop, args, save_past_dict, log_item)

    q_init[5] += 0.1
    moveJ(args, robot, q_init)
    # and now we can actually run
#    log_dict, final_iteration = loop_manager.run()

    #plotFromDict(log_dict, args)
    # plotting is now initiated in stophandler because then we get the plot 
    # even if we end sooner
    # TODO: add some math to analyze path precision

    


