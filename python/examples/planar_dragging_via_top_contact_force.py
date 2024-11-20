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
from ur_simple_control.visualize.visualize import plotFromDict
from ur_simple_control.basics.basics import moveJ

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
    parser.add_argument('--visualize', action=argparse.BooleanOptionalAction, 
            help="whether you want to visualize with gepetto, but \
                    NOTE: not implemented yet", default=False)
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
            help="maximum allowable iteration number (it runs at 500Hz)", default=50000)
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
            default=0.007)
    # TODO add low pass filtering and make it's parameters arguments too
    #######################################################################
    #                       task specific arguments                       #
    #######################################################################
    # TODO measure this for the new board
    parser.add_argument('--board-width', type=float, \
            help="width of the board (in meters) the robot will write on", \
            default=0.5)
    parser.add_argument('--board-height', type=float, \
            help="height of the board (in meters) the robot will write on", \
            default=0.35)
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
    args = parser.parse_args()
    if args.gripper and args.simulation:
        raise NotImplementedError('Did not figure out how to put the gripper in \
                the simulation yet, sorry :/ . You can have only 1 these flags right now')
    return args




#######################################################################
#                            control loop                             #
#######################################################################

# feedforward velocity, feedback position and force for impedance
def controller():
    pass


# control loop to be passed to ControlLoopManager
def controlLoopPlanarDragging(dmp, tc, controller, robot, i, past_data):
    breakFlag = False
    # TODO rename this into something less confusing
    save_past_dict = {}
    log_item = {}
    dmp.step(robot.dt) # dmp step
    # temporal coupling step
    tau = dmp.tau + tc.update(dmp, robot.dt) * robot.dt
    dmp.set_tau(tau)
    q = robot.getQ()
    Z = np.diag(np.ones(6))

    wrench = robot.getWrench()
    wrench = np.average(np.array(past_data['wrench']), axis=0)

    # first-order low pass filtering 
    # beta is a smoothing coefficient, smaller values smooth more, has to be in [0,1]
    beta = 0.007
    #wrench = beta * wrench + (1 - beta) * past_data['wrench'][-1]

    wrench = robot.getMtool().toDualActionMatrix().T @ wrench
    wrench = Z @ wrench
    # deepcopy for good coding practise (and correctness here)
    save_past_dict['wrench'] = copy.deepcopy(wrench)
    # rolling average
    if i % 100 == 0:
        print(wrench)
    J = pin.computeJointJacobian(robot.model, robot.data, q, robot.JOINT_ID)
    dq = robot.getQd()[:6].reshape((6,1))
    # get joitn 
    tau = J.T @ wrench
    tau = tau[:6].reshape((6,1))
    # compute control law:
    #vel_cmd = dmp.vel + args.kp * (dmp.pos - q[:6].reshape((6,1))) + args.alpha * tau
    vel_cmd = np.zeros(6)
    robot.sendQd(vel_cmd)

    # TODO find a better criterion for stopping
    if (np.linalg.norm(dmp.vel) < 0.0001) and (i > 5000):
        breakFlag = True
    # immediatelly stop if something weird happened (some non-convergence)
    if np.isnan(vel_cmd[0]):
        breakFlag = True

    # log what you said you'd log
    # TODO fix the q6 situation (hide this)
    log_item['wrench'] = wrench[:6].reshape((6,))

    return breakFlag, save_past_dict, log_item

if __name__ == "__main__":
    #######################################################################
    #                           software setup                            #
    #######################################################################
    args = getArgs()
    clikController = getClikController(args)
    robot = RobotManager(args)

    # calibrate FT first
    speed_down = np.array([0,0,-1,0,0,0])
    moveUntilContact(args, robot, speed_down)
    m_goal_up = robot.getMtool()
    m_goal_up.translation[2] += 0.1
    moveL(args, robot, m_goal_up)
    
    # controlLoopManager will populate the queue with these initial values
#    save_past_dict = {
#            'wrench' : np.zeros(6),
#        }
#    # here you give it it's initial value
#    log_dict = {
#            'wrench': np.zeros((args.max_iterations, 6)),
#        }
#    controlLoop = partial(controlLoopPlanarDragging, dmp, tc, controller, robot)
#    loop_manager = ControlLoopManager(robot, controlLoop, args, save_past_dict, log_dict)
#    #######################################################################
#    #                           physical setup                            #
#    #######################################################################
#    # TODO: add marker picking
#    # get up from the board
#    current_pose = robot.getMtool()
#    # and now we can actually run
#    log_dict, final_iteration = loop_manager.run()
#    mtool = robot.getMtool()
#    mtool.translation[1] = mtool.translation[1] - 0.1
#    moveL(args, robot, mtool)

    plotFromDict(log_dict, args)
    robot.stopHandler(None, None)
    robot.stopHandler(None, None)
    robot.stopHandler(None, None)
    # plot results
    plotFromDict(log_dict, args)
    # TODO: add some math to analyze path precision
