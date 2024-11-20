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
    parser.add_argument('--robot-ip', type=str, 
            help="robot's ip address", \
                    default="192.168.1.102")
    parser.add_argument('--pinocchio-only', action=argparse.BooleanOptionalAction, 
            help="whether you want to just integrate with pinocchio.\
                    NOTE: doesn't actually work because it's not a physics simulator", \
                    default=False)
    parser.add_argument('--visualize-manipulator', action=argparse.BooleanOptionalAction, 
            help="whether you want to visualize the manipulator and workspace with meshcat", default=False)
    parser.add_argument('--real-time-plotting', action=argparse.BooleanOptionalAction, 
            help="whether you want to have some real-time matplotlib graphs (parts of log_dict you select)", default=False)
    parser.add_argument('--gripper', type=str, \
            help="gripper you're using (no gripper is the default)", 
                        default="none", choices=['none', 'robotiq', 'onrobot'])
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
    parser.add_argument('--cartesian-space-impedance', action=argparse.BooleanOptionalAction, \
            help="is the impedance computed and added in cartesian or in joint space", default=False)
    parser.add_argument('--past-window-size', type=int, \
            help="how many timesteps of past data you want to save", default=5)
    parser.add_argument('--goal-error', type=float, \
            help="the final position error you are happy with. NOTE: not used here", \
            default=1e-3)
    # TODO: test the interaction of this and the overall demo
    parser.add_argument("--start-from-current-pose", action=argparse.BooleanOptionalAction, \
            help="if connected to the robot, read the current pose and set it as the initial pose for the robot. \
                 very useful and convenient when running simulation before running on real", \
                         default=False)
    parser.add_argument('--tikhonov-damp', type=float, \
            help="damping scalar in tikhonov regularization.\
            This is used when generating the joint trajectory from the drawing.", \
            default=1e-2)
    # TODO add the rest
    parser.add_argument('--clik-controller', type=str, \
            help="select which click algorithm you want", \
            default='dampedPseudoinverse', \
            choices=['dampedPseudoinverse', 'jacobianTranspose', 'invKinmQP'])
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


#######################################################################
#                            control loop                             #
#######################################################################

# feedforward velocity, feedback position and force for impedance
def controller():
    pass

# control loop to be passed to ControlLoopManager
def controlLoopPointImpedance(q_init, controller, robot : RobotManager, i, past_data):
    breakFlag = False
    # TODO rename this into something less confusing
    save_past_dict = {}
    log_item = {}
    q = robot.getQ()
    Mtool = robot.getT_w_e()
    wrench = robot.getWrench()
    log_item['wrench_raw'] = wrench.reshape((6,))
    # deepcopy for good coding practise (and correctness here)
    save_past_dict['wrench'] = copy.deepcopy(wrench)
    # rolling average
    #wrench = np.average(np.array(past_data['wrench']), axis=0)
    # first-order low pass filtering instead
    # beta is a smoothing coefficient, smaller values smooth more, has to be in [0,1]
    wrench = args.beta * wrench + (1 - args.beta) * past_data['wrench'][-1]
    Z = np.diag(np.array([1.0, 1.0, 2.0, 1.0, 1.0, 1.0]))
    #Z = np.diag(np.ones(6))

    wrench = Z @ wrench
    # this jacobian starts at the end of the end-effector.
    # but the wrench is probably modified wrt tip of the end effector
    # because we defined the load in the ur inteface thing.
    # so that might cause some offset.
    # test for this:
    # when i touch the f/t sensor directly, it gives on linear force,
    # when i touch it at the tip of the end-effector, then it give both linear force
    # and torque. so that means there's no memeing going on with internally transforming the wrench.
    # plus the f/t sensor has no way of knowing where the contact is anyway, so you can't account 
    # for this even if you wanted to.
    
    # this jacobian might be wrong
    J = pin.computeJointJacobian(robot.model, robot.data, q, robot.JOINT_ID)
    dq = robot.getQd()[:6].reshape((6,1))
    # get joint 
    tau = J.T @ wrench
#    if i % 25 == 0:
#        print(*tau.round(1))
    tau = tau[:6].reshape((6,1))
    # compute control law:
    # - feedback the position 
    # kv is not needed if we're running velocity control
    vel_cmd = args.kp * (q_init[:6].reshape((6,1)) - q[:6].reshape((6,1))) + args.alpha * tau
    #vel_cmd = np.zeros(6)
    robot.sendQd(vel_cmd)

    # immediatelly stop if something weird happened (some non-convergence)
    if np.isnan(vel_cmd[0]):
        breakFlag = True

    # log what you said you'd log
    # TODO fix the q6 situation (hide this)
    log_item['qs'] = q[:6].reshape((6,))
    log_item['dqs'] = dq.reshape((6,))
    log_item['wrench_used'] = wrench.reshape((6,))

    return breakFlag, save_past_dict, log_item



def controlLoopCartesianPointImpedance(Mtool_init, clik_controller, robot, i, past_data):
    breakFlag = False
    # TODO rename this into something less confusing
    save_past_dict = {}
    log_item = {}
    q = robot.getQ()
    Mtool = robot.getT_w_e()
    wrench = robot.getWrench()
    log_item['wrench_raw'] = wrench.reshape((6,))
    save_past_dict['wrench'] = copy.deepcopy(wrench)
    wrench = args.beta * wrench + (1 - args.beta) * past_data['wrench'][-1]
    # good generic values
    #Z = np.diag(np.array([1.0, 1.0, 2.0, 1.0, 1.0, 1.0]))
    # but let's stick to the default for now
    #Z = np.diag(np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0]))
    Z = np.diag(np.array([1.0, 1.0, 1.0, 10.0, 10.0, 10.0]))

    wrench = Z @ wrench
    # this jacobian starts at the end of the end-effector.
    # but the wrench is probably modified wrt tip of the end effector
    # because we defined the load in the ur inteface thing.
    # so that might cause some offset.
    # test for this:
    # when i touch the f/t sensor directly, it gives on linear force,
    # when i touch it at the tip of the end-effector, then it give both linear force
    # and torque. so that means there's no memeing going on with internally transforming the wrench.
    # plus the f/t sensor has no way of knowing where the contact is anyway, so you can't account 
    # for this even if you wanted to.
    J = pin.computeJointJacobian(robot.model, robot.data, q, robot.JOINT_ID)

    SEerror = Mtool.actInv(Mtool_init)
    err_vector = pin.log6(SEerror).vector 
    v_body_cmd = args.kp * err_vector + args.alpha * wrench
    # now this v_spatial_cmd can be passed to any inverse kinematics algorithm

    vel_cmd = clik_controller(J, v_body_cmd)
    robot.sendQd(vel_cmd)

    # immediatelly stop if something weird happened (some non-convergence)
    if np.isnan(vel_cmd[0]):
        breakFlag = True

    dq = robot.getQd()[:6].reshape((6,1))
    # log what you said you'd log
    # TODO fix the q6 situation (hide this)
    log_item['qs'] = q[:6].reshape((6,))
    log_item['dqs'] = dq.reshape((6,))
    log_item['wrench_used'] = wrench.reshape((6,))

    return breakFlag, save_past_dict, log_item


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
    
    # TODO and NOTE the weight, TCP and inertial matrix needs to be set on the robot
    # you already found an API in rtde_control for this, just put it in initialization 
    # under using/not-using gripper parameters
    # ALSO NOTE: to use this you need to change the version inclusions in
    # ur_rtde due to a bug there in the current ur_rtde + robot firmware version 
    # (the bug is it works with the firmware verion, but ur_rtde thinks it doesn't)
    # here you give what you're saving in the rolling past window 
    # it's initial value.
    # controlLoopManager will populate the queue with these initial values
    save_past_dict = {
            'wrench' : np.zeros(6),
        }
    # here you give it it's initial value
    log_item = {
            'qs' : np.zeros(robot.n_arm_joints),
            'dqs' : np.zeros(robot.n_arm_joints),
            'wrench_raw' : np.zeros(6),
            'wrench_used' : np.zeros(6),
        }
    q_init = robot.getQ()
    Mtool_init = robot.getT_w_e()

    if not args.cartesian_space_impedance:
        controlLoop = partial(controlLoopPointImpedance, q_init, controller, robot)
    else:
        controlLoop = partial(controlLoopCartesianPointImpedance, Mtool_init, clikController, robot)

    loop_manager = ControlLoopManager(robot, controlLoop, args, save_past_dict, log_item)

    #moveJ(args, robot, dmp.pos.reshape((6,)))
    # and now we can actually run
    log_dict, final_iteration = loop_manager.run()

    #plotFromDict(log_dict, args)
    # plotting is now initiated in stophandler because then we get the plot 
    # even if we end sooner
    loop_manager.stopHandler(None, None)
    # TODO: add some math to analyze path precision

    


