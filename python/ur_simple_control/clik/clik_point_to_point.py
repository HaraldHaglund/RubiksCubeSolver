import pinocchio as pin
import numpy as np
import copy
import argparse
from functools import partial
from ur_simple_control.managers import ControlLoopManager, RobotManager
import time
from qpsolvers import solve_qp

def get_args():
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
    parser = argparse.ArgumentParser(description='Run closed loop inverse kinematics \
            of various kinds. Make sure you know what the goal is before you run!',
            formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--simulation', action=argparse.BooleanOptionalAction, 
            help="whether you are running the UR simulator", default=False)
    parser.add_argument('--robot-ip', type=str, 
            help="robot's ip address", \
                    default="192.168.1.102")
    parser.add_argument('--debug_prints', action=argparse.BooleanOptionalAction, 
            help="print some info for debugging", default=False)
    parser.add_argument('--pinocchio-only', action=argparse.BooleanOptionalAction, 
            help="whether you want to just integrate with pinocchio", default=False)
    parser.add_argument('--visualize-manipulator', action=argparse.BooleanOptionalAction, 
            help="whether you want to visualize the manipulator and workspace with meshcat", default=False)
    parser.add_argument('--real-time-plotting', action=argparse.BooleanOptionalAction, 
            help="whether you want to have some real-time matplotlib graphs (parts of log_dict you select)", default=False)
    parser.add_argument('--gripper', action=argparse.BooleanOptionalAction, \
            help="whether you're using the gripper", default=False)
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
    parser.add_argument('--tikhonov-damp', type=float, \
            help="damping scalar in tikhonov regularization", default=1e-3)
    # TODO add the rest
    parser.add_argument('--clik-controller', type=str, \
            help="select which click algorithm you want", \
            default='dampedPseudoinverse', choices=['dampedPseudoinverse', 'jacobianTranspose'])
        # maybe you want to scale the control signal
    parser.add_argument('--controller-speed-scaling', type=float, \
            default='1.0', help='not actually_used atm')
    parser.add_argument('--alpha', type=float, \
            help="force feedback proportional coefficient", \
            #default=0.01)
            default=0.05)
    parser.add_argument('--beta', type=float, \
            help="low-pass filter beta parameter", \
            default=0.01)

    args = parser.parse_args()
    if args.gripper and args.simulation:
        raise NotImplementedError('Did not figure out how to put the gripper in \
                the simulation yet, sorry :/ . You can have only 1 these flags right now')
    return args

#######################################################################
#                             controllers                             #
#######################################################################
"""
controllers general
-----------------------
really trully just the equation you are running.
ideally, everything else is a placeholder,
meaning you can swap these at will.
if a controller has some additional arguments,
those are put in via functools.partial in getClikController,
so that you don't have to worry about how your controller is handled in
the actual control loop after you've put it in getClikController,
which constructs the controller from an argument to the file.
"""

def dampedPseudoinverse(tikhonov_damp, J, err_vector):
    qd = J.T @ np.linalg.inv(J @ J.T + np.eye(J.shape[0], J.shape[0]) * tikhonov_damp) @ err_vector
    return qd

def jacobianTranspose(J, err_vector):
    qd = J.T @ err_vector
    return qd

def invKinmQP(J, err_vector):
    # maybe a lower precision dtype is equally good, but faster?
    P = np.eye(J.shape[1], dtype="double")
    # TODO: why is q all 0?
    q = np.array([0] * J.shape[1], dtype="double")
    G = None
    # TODO: extend for orientation as well
    b = err_vector#[:3]
    A = J#[:3]
    # TODO: you probably want limits here
    lb = None
    ub = None
    h = None
    #qd = solve_qp(P, q, G, h, A, b, lb, ub, solver="ecos")
    qd = solve_qp(P, q, G, h, A, b, lb, ub, solver="quadprog")
    return qd

def getClikController(args):
    """
    getClikController
    -----------------
    A string argument is used to select one of these.
    It's a bit ugly, bit totally functional and OK solution.
    we want all of theme to accept the same arguments, i.e. the jacobian and the error vector.
    if they have extra stuff, just map it in the beginning with partial
    NOTE: this could be changed to something else if it proves inappropriate later
    TODO: write out other algorithms
    """
    if args.clik_controller == "dampedPseudoinverse":
        return partial(dampedPseudoinverse, args.tikhonov_damp)
    if args.clik_controller == "jacobianTranspose":
        return jacobianTranspose
    # TODO implement and add in the rest
    #if controller_name == "invKinmQPSingAvoidE_kI":
    #    return invKinmQPSingAvoidE_kI
    #if controller_name == "invKinm_PseudoInv":
    #    return invKinm_PseudoInv
    #if controller_name == "invKinm_PseudoInv_half":
    #    return invKinm_PseudoInv_half
    if args.clik_controller == "invKinmQP":
        return invKinmQP
    #if controller_name == "invKinmQPSingAvoidE_kI":
    #    return invKinmQPSingAvoidE_kI
    #if controller_name == "invKinmQPSingAvoidE_kM":
    #    return invKinmQPSingAvoidE_kM
    #if controller_name == "invKinmQPSingAvoidManipMax":
    #    return invKinmQPSingAvoidManipMax

    # default
    return partial(dampedPseudoinverse, args.tikhonov_damp)


# modularity yo
# past data to comply with the API
# TODO make this a kwarg or whatever to be more lax with this
def controlLoopClik(robot, clik_controller, i, past_data):
    """
    controlLoopClik
    ---------------
    generic control loop for clik (handling error to final point etc).
    in some version of the universe this could be extended to a generic
    point-to-point motion control loop.
    """
    breakFlag = False
    log_item = {}
    # know where you are, i.e. do forward kinematics
    q = robot.getQ()
    # first check whether we're at the goal
    SEerror = robot.data.oMi[robot.JOINT_ID].actInv(robot.Mgoal)
    err_vector = pin.log6(SEerror).vector 
    if np.linalg.norm(err_vector) < robot.args.goal_error:
#      print("Convergence achieved, reached destionation!")
        breakFlag = True
    # pin.computeJointJacobian is much different than the C++ version lel
    # TODO: put in step, havew getter for it
    J = pin.computeJointJacobian(robot.model, robot.data, q, robot.JOINT_ID)
    # compute the joint velocities.
    # just plug and play different ones
    qd = clik_controller(J, err_vector)
    robot.sendQd(qd)
    
    # TODO: only print under a debug flag, it's just clutter otherwise
    # we do the printing here because controlLoopManager should be general.
    # and these prints are click specific (although i'm not 100% happy with the arrangement)
#    if not i % 1000:
#        print("pos", robot.data.oMi[robot.JOINT_ID])
#        print("linear error = ", pin.log6(SEerror).linear)
#        print("angular error = ", pin.log6(SEerror).angular)
#        print(" error = ", err_vector.transpose())

    log_item['qs'] = q.reshape((robot.model.nq,))
    # get actual velocity, not the commanded one.
    # although that would be fun to compare i guess
    # TODO: have both, but call the compute control signal like it should be
    log_item['dqs'] = robot.getQd().reshape((robot.model.nq,))
    # we're not saving here, but need to respect the API, 
    # hence the empty dict
    return breakFlag, {}, log_item


def moveUntilContactControlLoop(contact_detecting_force, speed, robot, clik_controller, i, past_data):
    """
    moveUntilContactControlLoop
    ---------------
    generic control loop for clik (handling error to final point etc).
    in some version of the universe this could be extended to a generic
    point-to-point motion control loop.
    """
    breakFlag = False
    # know where you are, i.e. do forward kinematics
    q = robot.getQ()
    # break if wrench is nonzero basically
    #wrench = robot.getWrench()
    # you're already giving the speed in the EE i.e. body frame
    # so it only makes sense to have the wrench in the same frame
    wrench = robot.getWrenchInEE()
    # and furthermore it's a reasonable assumption that you'll hit the thing
    # in the direction you're going in.
    # thus we only care about wrenches in those direction coordinates
    mask = speed != 0.0
    # NOTE: contact getting force is a magic number
    # it is a 100% empirical, with the goal being that it's just above noise.
    # so far it's worked fine, and it's pretty soft too.
    if np.linalg.norm(wrench[mask]) > contact_detecting_force:
        print("hit with", np.linalg.norm(wrench[mask]))
        breakFlag = True
    # pin.computeJointJacobian is much different than the C++ version lel
    J = pin.computeJointJacobian(robot.model, robot.data, q, robot.JOINT_ID)
    # compute the joint velocities.
    qd = clik_controller(J, speed)
    robot.sendQd(qd)
    return breakFlag, {}, {}

def moveUntilContact(args, robot, speed):
    """
    moveUntilContact
    -----
    does clik until it feels something with the f/t sensor
    """
    assert type(speed) == np.ndarray 
    clik_controller = getClikController(args)
    # TODO: just make the manager pass the robot or something, this is weird man
    controlLoop = partial(moveUntilContactControlLoop, args.contact_detecting_force, speed, robot, clik_controller)
    # we're not using any past data or logging, hence the empty arguments
    loop_manager = ControlLoopManager(robot, controlLoop, args, {}, {})
    log_dict, final_iteration = loop_manager.run()
    # TODO: remove, this isn't doing anything
    time.sleep(0.01)
    print("Collision detected!!")

def moveL(args, robot, goal_point):
    """
    moveL
    -----
    does moveL.
    send a SE3 object as goal point.
    if you don't care about rotation, make it np.zeros((3,3))
    """
    assert type(goal_point) == pin.pinocchio_pywrap.SE3
    robot.Mgoal = copy.deepcopy(goal_point)
    clik_controller = getClikController(args)
    controlLoop = partial(controlLoopClik, robot, clik_controller)
    # we're not using any past data or logging, hence the empty arguments
    log_item = {
            'qs' : np.zeros(robot.model.nq),
            'dqs' : np.zeros(robot.model.nq),
        }
    loop_manager = ControlLoopManager(robot, controlLoop, args, {}, log_item)
    log_dict, final_iteration = loop_manager.run()
    # TODO: remove, this isn't doing anything
    time.sleep(0.01)
    print("MoveL done: convergence achieved, reached destionation!")


def controlLoopCompliantClik(args, robot : RobotManager, i, past_data):
    """
    controlLoopClik
    ---------------
    generic control loop for clik (handling error to final point etc).
    in some version of the universe this could be extended to a generic
    point-to-point motion control loop.
    """
    breakFlag = False
    log_item = {}
    save_past_dict = {}
    # know where you are, i.e. do forward kinematics
    q = robot.getQ()
    T_w_e = robot.getT_w_e()
    wrench = robot.getWrench()
    # we need to overcome noise if we want to converge
    if np.linalg.norm(wrench) < args.minimum_detectable_force_norm:
        wrench = np.zeros(6)
    save_past_dict['wrench'] = copy.deepcopy(wrench)
    wrench = args.beta * wrench + (1 - args.beta) * past_data['wrench'][-1]
    mapping = np.zeros((6,6))
    mapping[0:3, 0:3] = T_w_e.rotation
    mapping[3:6, 3:6] = T_w_e.rotation
    wrench = mapping.T @ wrench
    #Z = np.diag(np.array([1.0, 1.0, 2.0, 1.0, 1.0, 1.0]))
    Z = np.diag(np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0]))
    wrench = Z @ wrench
    #pin.forwardKinematics(robot.model, robot.data, q)
    # first check whether we're at the goal
    SEerror = T_w_e.actInv(robot.Mgoal)
    err_vector = pin.log6(SEerror).vector 
    if np.linalg.norm(err_vector) < robot.args.goal_error:
#      print("Convergence achieved, reached destionation!")
        breakFlag = True
    # pin.computeJointJacobian is much different than the C++ version lel
    J = pin.computeJointJacobian(robot.model, robot.data, q, robot.JOINT_ID)
    # compute the joint velocities.
    # just plug and play different ones
    qd = J.T @ np.linalg.inv(J @ J.T + np.eye(J.shape[0], J.shape[0]) * args.tikhonov_damp) @ err_vector
    tau = J.T @ wrench
    #tau = tau[:6].reshape((6,1))
    qd += args.alpha * tau
    robot.sendQd(qd)
    
    log_item['qs'] = q.reshape((robot.model.nq,))
    # get actual velocity, not the commanded one.
    # although that would be fun to compare i guess
    # TODO: have both, but call the compute control signal like it should be
    log_item['dqs'] = robot.getQd().reshape((robot.model.nq,))
    log_item['wrench'] = wrench.reshape((6,))
    log_item['tau'] = tau.reshape((robot.model.nq,))
    # we're not saving here, but need to respect the API, 
    # hence the empty dict
    return breakFlag, save_past_dict, log_item

# add a threshold for the wrench
def compliantMoveL(args, robot, goal_point):
    """
    compliantMoveL
    -----
    does compliantMoveL - a moveL, but with compliance achieved
    through f/t feedback
    send a SE3 object as goal point.
    if you don't care about rotation, make it np.zeros((3,3))
    """
#    assert type(goal_point) == pin.pinocchio_pywrap.SE3
    robot.Mgoal = copy.deepcopy(goal_point)
    clik_controller = getClikController(args)
    controlLoop = partial(controlLoopCompliantClik, args, robot)
    # we're not using any past data or logging, hence the empty arguments
    log_item = {
            'qs' : np.zeros(robot.model.nq),
            'wrench' : np.zeros(6),
            'tau' : np.zeros(robot.model.nq),
            'dqs' : np.zeros(robot.model.nq),
        }
    save_past_dict = {
            'wrench': np.zeros(6),
            }
    loop_manager = ControlLoopManager(robot, controlLoop, args, save_past_dict, log_item)
    log_dict, final_iteration = loop_manager.run()
    # TODO: remove, this isn't doing anything
    time.sleep(0.01)
    print("compliantMoveL done: convergence achieved, reached destionation!")
    return log_dict, final_iteration

if __name__ == "__main__": 
    args = get_args()
    robot = RobotManager(args)
    Mgoal = robot.defineGoalPointCLI()
    clik_controller = getClikController(args)
    controlLoop = partial(controlLoopClik, robot, clik_controller)
    # we're not using any past data or logging, hence the empty arguments
    loop_manager = ControlLoopManager(robot, controlLoop, args, {}, {})
    log_dict, final_iteration = loop_manager.run()
