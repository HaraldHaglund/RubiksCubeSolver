#, this is the main file for the drawing your drawings with a dmp with force feedback
# TODO:
# 2. delete the unnecessary comments
# 8. add some code to pick up the marker from a prespecified location
# 10. write documentation as you go along

import pinocchio as pin
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import copy
import argparse
import time
from functools import partial
from ur_simple_control.visualize.visualize import plotFromDict
from ur_simple_control.util.draw_path import drawPath
from ur_simple_control.dmp.dmp import DMP, NoTC,TCVelAccConstrained 
# TODO merge these clik files as well, they don't deserve to be separate
# TODO but first you need to clean up clik.py as specified there
from ur_simple_control.clik.clik_point_to_point import getClikController, moveL, moveUntilContact, controlLoopClik, compliantMoveL
from ur_simple_control.clik.clik_trajectory_following import map2DPathTo3DPlane, clikCartesianPathIntoJointPath
from ur_simple_control.managers import ControlLoopManager, RobotManager
from ur_simple_control.util.calib_board_hacks import calibratePlane, getSpeedInDirectionOfN
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
    parser.add_argument('--robot-ip', type=str, 
            help="robot's ip address (only needed if running on the real robot)", \
                    default="192.168.1.102")
    parser.add_argument('--pinocchio-only', action=argparse.BooleanOptionalAction, \
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
    # TODO: make this optional
    parser.add_argument('--max-iterations', type=int, \
            help="maximum allowable iteration number (it runs at 500Hz)", default=50000)
    parser.add_argument('--debug-prints', action=argparse.BooleanOptionalAction, \
            help="print some debug info", default=False)
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
    parser.add_argument("--start-from-current-pose", action=argparse.BooleanOptionalAction, \
            help="if connected to the robot, read the current pose and set it as the initial pose for the robot.\
                 very useful and convenient when running simulation before running on real", \
                         default=False)
    parser.add_argument('--tikhonov-damp', type=float, \
            help="damping scalar in tikhonov regularization.\
            This is used when generating the joint trajectory from the drawing.", \
            default=1e-2)
    parser.add_argument('--minimum-detectable-force-norm', type=float, \
            help="we need to disregard noise to converge despite filtering. \
                  a quick fix is to zero all forces of norm below this argument threshold.",
                 default=3.0)
    # TODO add the rest
    parser.add_argument('--clik-controller', type=str, \
            help="select which click algorithm you want", \
            default='dampedPseudoinverse', \
            choices=['dampedPseudoinverse', 'jacobianTranspose'])
        # maybe you want to scale the control signal
    parser.add_argument('--controller-speed-scaling', type=float, \
            default='1.0', help='not actually_used atm')
    parser.add_argument('--contact-detecting-force', type=float, \
            default='1.3', help='the force used to detect contact (collision) in the moveUntilContact function')
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
            #default=0.05)
    parser.add_argument('--beta', type=float, \
            help="low-pass filter beta parameter", \
            default=0.01)
    # TODO add low pass filtering and make it's parameters arguments too
    #######################################################################
    #                       task specific arguments                       #
    #######################################################################
    # TODO measure this for the new board
    parser.add_argument('--board-width', type=float, \
            help="width of the board (in meters) the robot will write on", \
            #default=0.5)
            default=0.25)
    parser.add_argument('--board-height', type=float, \
            help="height of the board (in meters) the robot will write on", \
            #default=0.35)
            default=0.25)
    parser.add_argument('--board-wiping', action=argparse.BooleanOptionalAction, \
            help="are you wiping the board (default is no because you're writing)", \
            default=False)
    # TODO: add axis direction too!!!!!!!!!!!!!!!!!
    # and change the various offsets accordingly
    parser.add_argument('--board-axis', type=str, \
            choices=['z', 'y'], \
            help="(world) axis (direction) in which you want to search for the board", \
            default="z")
    parser.add_argument('--calibration', action=argparse.BooleanOptionalAction, \
            help="whether you want to do calibration", default=False)
    parser.add_argument('--draw-new', action=argparse.BooleanOptionalAction, \
            help="whether draw a new picture, or use the saved path path_in_pixels.csv", default=True)
    parser.add_argument('--pick-up-marker', action=argparse.BooleanOptionalAction, \
            help="""
    whether the robot should pick up the marker.
    NOTE: THIS IS FROM A PREDEFINED LOCATION.
    """, default=False)
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
    if args.calibration or args.pick_up_marker:
        args.gripper = True
    return args

# go and pick up the marker
# marker position:
"""
  R =
 -0.73032 -0.682357 0.0319574
-0.679774  0.730578 0.0645244
-0.067376 0.0253997 -0.997404
  p =  0.574534 -0.508343  0.249325

pin: 0.5745 -0.5083 0.2493 3.1161 0.0674 -2.392
ur5: 0.5796 -0.4982 0.0927 -1.1314 2.8725 0.0747
q: -0.5586 -2.3103 -1.1638 -1.2468 1.6492 0.262 0.0 0.0

"""
def getMarker(args, robot, rotation_matrix, translation_vector):
    # init position is the safe one above starting point of the board
    above_starting_write_point = pin.SE3.Identity()
    # start 20cm away from the board
    above_starting_write_point.translation[2] = -0.2
    transf_bord_to_board = pin.SE3(rotation_matrix, translation_vector)
    above_starting_write_point = transf_bord_to_board.act(above_starting_write_point)
    compliantMoveL(args, robot, above_starting_write_point)
    Tinit = robot.getT_w_e().copy()
    q_init = robot.getQ()
    #exit()
    #moveJ(args, robot, q_init)
    compliantMoveL(args, robot, above_starting_write_point)
    print("above_starting_write_point", above_starting_write_point)
    # goal position, predefined
    Tgoal = pin.SE3()
    Tgoal.rotation = np.array([
        [ -0.73032, -0.682357, 0.0319574],
        [-0.679774,  0.730578, 0.0645244],
        [-0.067376, 0.0253997, -0.997404]])
    Tgoal.translation = np.array([0.574534, -0.508343,  0.249325])

    # slighly up from goal to not hit marker weirdly
    TgoalUP = Tgoal.copy() 
    TgoalUP.translation += np.array([0,0,0.2])
    #TgoalUP.translation += np.array([0,0,0.3])


    print("going to above marker")
    robot.Mgoal = TgoalUP.copy()
    compliantMoveL(args, robot, TgoalUP.copy())
    #moveL(args, robot, TgoalUP.copy())
    #log_dict, final_iteration = loop_manager.run()
    print("going down to marker")
    robot.Mgoal = Tgoal.copy()
    compliantMoveL(args, robot, Tgoal.copy())
    #moveL(args, robot, Tgoal.copy())
    #log_dict, final_iteration = loop_manager.run()
    print("picking up marker")
    robot.closeGripper()
    time.sleep(2)
    print("going up")
    robot.Mgoal = TgoalUP.copy()
    compliantMoveL(args, robot, TgoalUP.copy())
    #moveL(args, robot, TgoalUP.copy())
    #log_dict, final_iteration = loop_manager.run()
    print("going back")
    # TODO: this HAS to be a movej
    # in fact all of them should be
    robot.Mgoal = Tinit.copy()
    #compliantMoveL(args, robot, Tinit.copy())
    moveJ(args, robot, q_init)
    #moveL(args, robot, Tinit.copy())
    #log_dict, final_iteration = loop_manager.run()
    print("got back")

"""
findMarkerOffset
---------------
This relies on having the correct orientation of the plane 
and the correct translation vector for top-left corner.
Idea is you pick up the marker, go to the top corner,
touch it, and see the difference between that and the translation vector.
Obviously it's just a hacked solution, but it works so who cares.
"""
def findMarkerOffset(args, robot, rotation_matrix, translation_vector, q_init):
    # TODO make this more general
    # so TODO: calculate TCP speed based on the rotation matrix
    # and then go
    above_starting_write_point = pin.SE3.Identity()
    #above_starting_write_point.translation[0] = args.board_width / 2
    # this one might be with a minus sign
    #above_starting_write_point.translation[1] = -1* args.board_height / 2
    # start 20cm away from the board
    above_starting_write_point.translation[2] = -0.2
    transf_bord_to_board = pin.SE3(rotation_matrix, translation_vector)
    above_starting_write_point = transf_bord_to_board.act(above_starting_write_point)
    print("above_starting_write_point", above_starting_write_point)
    print("current T_w_e", robot.getT_w_e())
    #exit()
    #moveJ(args, robot, q_init)
    compliantMoveL(args, robot, above_starting_write_point)
    if args.board_axis == 'z':
        axis_of_rot = rotation_matrix[:,2]
    elif args.board_axis == 'y':
        axis_of_rot = rotation_matrix[:,1]
    else:
        print("you passed", board_axis, ", but it has to be 'z' or 'y'")
        exit()
    # it's going out of the board, and we want to go into the board, right????
    # TODO test this
    #z_of_rot = z_of_rot 
    print("vector i'm following:", axis_of_rot)
    speed = getSpeedInDirectionOfN(rotation_matrix, args.board_axis)
    speed = np.zeros(6)
    # this is in the end-effector frame, so this means going straight down
    # because we are using the body jacobians in our clik
    # TODO: make this both more transparent AND provide the option to do clik
    # with a spatial jacobian
    speed[2] = 0.02
    #speed[2] = speed[2] * -1
    #robot.rtde_control.moveUntilContact(speed)
    moveUntilContact(args, robot, speed)
    # we use the pin coordinate system because that's what's 
    # the correct thing long term accross different robots etc
    current_translation = robot.getT_w_e().translation
    # i only care about the z because i'm fixing the path atm
    # but, let's account for the possible milimiter offset 'cos why not
    #print("translation_vector", translation_vector)
    #print("current_translation", current_translation)
    #print("translation_vector - current_translation", \
    #        translation_vector - current_translation)
    marker_offset = np.linalg.norm(translation_vector - current_translation)

    print("going back")
    # TODO: this HAS to be a movej
    # in fact all of them should be
    #robot.Mgoal = Tinit.copy()
    compliantMoveL(args, robot, above_starting_write_point)
#    robot.setSpeedSlider(old_speed_slider)
    return marker_offset

#######################################################################
#                            control loop                             #
#######################################################################

# feedforward velocity, feedback position and force for impedance
# TODO: actually write this out
def controller():
    pass

# TODO:
# regarding saving data you have 2 options:
# 1) explicitely return what you want to save - you can't magically read local variables
# 2) make controlLoop a class and then save handle the saving behind the scenes -
#    now you these variables are saved in a class so they're not local variables
# option 1 is clearly more hands-on and thus worse
# option 2 introduces a third big class and makes everything convoluted.
# for now, we go for option 1) because it's simpler to implement and deal with.
# but in the future, implementing 2) should be tried. you really have to try 
# to do it cleanly to see how good/bad it is.
# in that case you'd most likely want to inherit ControlLoopManager and go from there.
# you'd still need to specify what you're saving with self.that_variable so no matter
# there's no running away from that in any case.
# it's 1) for now, it's the only non-idealy-clean part of this solution, and it's ok really.
# TODO but also look into something fancy like some decorator or something and try
# to find option 3)

# control loop to be passed to ControlLoopManager
def controlLoopWriting(dmp, tc, controller, robot, i, past_data):
    breakFlag = False
    # TODO rename this into something less confusing
    save_past_dict = {}
    log_item = {}
    dmp.step(robot.dt) # dmp step
    # temporal coupling step
    tau_dmp = dmp.tau + tc.update(dmp, robot.dt) * robot.dt
    dmp.set_tau(tau_dmp)
    q = robot.getQ()
    T_w_e = robot.getT_w_e()
#    if args.board_axis == 'z':
#        Z = np.diag(np.array([0.0, 0.0, 2.0, 1.0, 1.0, 1.0]))
#    if args.board_axis == 'y':
#        Z = np.diag(np.array([0.0, 1.0, 0.0, 1.0, 1.0, 1.0]))
    Z = np.diag(np.array([0.0, 0.0, 2.0, 1.0, 1.0, 1.0]))
    #Z = np.diag(np.ones(6))
    #Z = np.diag(np.array([0.1, 0.1, 1.0, 0.1, 0.1, 0.1]))

    #Z = np.diag(np.array([1.0, 0.6, 1.0, 0.5, 0.5, 0.5]))
    wrench = robot.getWrench()
    # deepcopy for good coding practise (and correctness here)
    save_past_dict['wrench'] = copy.deepcopy(wrench)
    # rolling average
    #wrench = np.average(np.array(past_data['wrench']), axis=0)

    # first-order low pass filtering instead
    # beta is a smoothing coefficient, smaller values smooth more, has to be in [0,1]
    wrench = args.beta * wrench + (1 - args.beta) * past_data['wrench'][-1]

    # it's just coordinate change from base to end-effector,
    # they're NOT the same rigid body,
    # the vector itself is not changing, only the coordinate representation

    wrench = Z @ wrench
    J = pin.computeJointJacobian(robot.model, robot.data, q, robot.JOINT_ID)
    dq = robot.getQd()[:6].reshape((6,1))
    # get joint 
    tau = J.T @ wrench
    tau = tau[:6].reshape((6,1))
    # compute control law:
    # - feedforward the velocity and the force reading
    # - feedback the position 
    # TODO: don't use vel for qd, it's confusion (yes, that means changing dmp code too)
    # TODO: put this in a controller function for easy swapping (or don't if you won't swap)
    # solve this q[:6] bs (clean it up)
    vel_cmd = dmp.vel + args.kp * (dmp.pos - q[:6].reshape((6,1))) + args.alpha * tau
    robot.sendQd(vel_cmd)

    # tau0 is the minimum time needed for dmp
    # 500 is the frequency
    # so we need tau0 * 500 iterations minimum
    if (np.linalg.norm(dmp.vel) < 0.01) and (i > int(args.tau0 * 500)):
        breakFlag = True
    # immediatelly stop if something weird happened (some non-convergence)
    if np.isnan(vel_cmd[0]):
        breakFlag = True

    # log what you said you'd log
    # TODO fix the q6 situation (hide this)
    log_item['qs'] = q[:6].reshape((6,))
    log_item['dmp_poss'] = dmp.pos.reshape((6,))
    log_item['dqs'] = dq.reshape((6,))
    log_item['dmp_vels'] = dmp.vel.reshape((6,))
    log_item['wrench'] = wrench.reshape((6,))
    log_item['tau'] = tau.reshape((6,))

    return breakFlag, save_past_dict, log_item

if __name__ == "__main__":

    #######################################################################
    #                           software setup                            #
    #######################################################################
    args = getArgs()
    print(args)
    clikController = getClikController(args)
    robot = RobotManager(args)

    # calibrate FT first
    # it's done by default now because it's basically always necessary

    #######################################################################
    #          drawing a path, making a joint trajectory for it           #
    #######################################################################
    # TODO make these ifs make more sense
    
    # draw the path on the screen
    if args.draw_new:
        # pure evil way to solve a bug that was pure evil
        matplotlib.use('tkagg')
        pixel_path = drawPath(args)
        matplotlib.use('qtagg')
        # make it 3D
    else:
        pixel_path_file_path = './path_in_pixels.csv'
        pixel_path = np.genfromtxt(pixel_path_file_path, delimiter=',')
    # do calibration if specified
    if args.calibration:
        rotation_matrix, translation_vector, q_init = \
            calibratePlane(args, robot, args.board_width, args.board_height, \
                           args.n_calibration_tests)
        print(q_init)
        print(rotation_matrix)
        print(translation_vector)
        #exit()
    else:
        # TODO: save this somewhere obviously
        # also make it prettier if possible
        print("using predefined values")
        #q_init = np.array([1.4545, -1.7905, -1.1806, -1.0959, 1.6858, -0.1259, 0.0, 0.0])
        #translation_vector = np.array([0.10125722 ,0.43077874 ,0.9110792 ])
        #rotation_matrix = np.array([[1.  ,       0.         ,0.00336406],
        #                            [-0.        , -0.00294646,  0.99999   ],
        #                            [ 0.        , -0.99999  ,  -0.00294646]])
        #translation_vector = np.array([0.21997482, 0.41345861, 0.7314353])
        #rotation_matrix = np.array([
        #                            [ 1.,          0.,          0.01792578],
        #                            [ 0.,         -0.58973106,  0.80740073],
        #                            [ 0.,         -0.80740073, -0.58973106]]) 
        #q_init = np.array([ 1.33085752e+00, -1.44578363e+00, -1.21776414e+00, -1.17214762e+00,   1.75202715e+00, -1.94359605e-01,  2.94117647e-04,  2.94117647e-04])
        translation_vector = np.array([0.21754368, 0.42021616, 0.74162252])
        rotation_matrix = np.array([[ 1. ,         0.          ,0.0139409 ],
                                     [ 0.,         -0.61730976 , 0.78659666],
                                     [ 0.,         -0.78659666 ,-0.61730976]]) 
        q_init = np.array([ 1.32022870e+00, -1.40114772e+00, -1.27237797e+00, -1.15715911e+00,
  1.76543856e+00, -2.38652054e-01,  2.94117647e-04,  2.94117647e-04])
        #q_init = robot.getQ()
        #T_w_e = robot.getT_w_e()
        #rotation_matrix = np.array([
        #                        [-1, 0, 0],
        #                        [0, 0, -1],
        #                        [0, -1, 0]])
        #translation_vector = T_w_e.translation.copy()

    # make the path 3D
    path = map2DPathTo3DPlane(pixel_path, args.board_width, args.board_height)
    # TODO: fix and trust z axis in 2D to 3D path
    # TODO: add an offset of the marker (this is of course approximate)
    # TODO: make this an argument once the rest is OK
    # ---> just go to the board while you have the marker ready to find this
    # ---> do that right here
    if args.pick_up_marker:
        getMarker(args, robot, rotation_matrix, translation_vector)

    if args.find_marker_offset:
        # find the marker offset
        # TODO find a better q init (just moveL away from the board)
        # THIS Q_INIT IS NOT SUITED FOR THIS PURPOSE!!!!!
        #marker_offset = findMarkerOffset(args, robot, rotation_matrix, translation_vector, q_init)
        marker_offset = findMarkerOffset(args, robot, rotation_matrix, translation_vector, q_init)
        print('marker_offset', marker_offset)
        # we're going in a bit deeper
        #path = path + np.array([0.0, 0.0, -1 * marker_offset + 0.015])
        if not args.board_wiping:
            #path = path + np.array([0.0, 0.0, -1 * marker_offset + 0.003])
            path = path + np.array([0.0, 0.0, -1 * marker_offset + 0.005])
        else:
            path = path + np.array([0.0, 0.0, -1 * marker_offset + 0.015])
    else:
        #path = path + np.array([0.0, 0.0, -0.1503])
        path = path + np.array([0.0, 0.0, - 0.092792+ 0.003])

    # and if you don't want to draw new nor calibrate, but you want the same path
    # with a different clik, i'm sorry, i can't put that if here.
    # atm running the same joint trajectory on the same thing makes for easier testing
    # of the final system.
    if args.draw_new or args.calibration or args.find_marker_offset:
        
        #path = path + np.array([0.0, 0.0, -0.0938])
        # create a joint space trajectory based on the 3D path
    # TODO: add flag of success (now it's your eyeballs and printing)
    # and immediatelly exit if it didn't work
        joint_trajectory = clikCartesianPathIntoJointPath(path, args, robot, \
            clikController, q_init, rotation_matrix, translation_vector)
    else:
        joint_trajectory_file_path = './joint_trajectory.csv'
        joint_trajectory = np.genfromtxt(joint_trajectory_file_path, delimiter=',')
    
    # create DMP based on the trajectory
    dmp = DMP(joint_trajectory)
    if not args.temporal_coupling:
        tc = NoTC()
    else:
        # TODO test whether this works (it should, but test it)
        # test the interplay between this and the speed slider
        # ---> SPEED SLIDER HAS TO BE AT 1.0
        v_max_ndarray = np.ones(robot.n_arm_joints) * robot.max_qd #* args.speed_slider
        # test the interplay between this and the speed slider
        # args.acceleration is the actual maximum you're using
        a_max_ndarray = np.ones(robot.n_arm_joints) * args.acceleration #* args.speed_slider
        tc = TCVelAccConstrained(args.gamma_nominal, args.gamma_a, v_max_ndarray, a_max_ndarray, args.eps_tc)

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
            'dmp_poss' : np.zeros(robot.n_arm_joints),
            'dqs' : np.zeros(robot.n_arm_joints),
            'dmp_vels' : np.zeros(robot.n_arm_joints),
            'wrench' : np.zeros(6),
            'tau' : np.zeros(robot.n_arm_joints),
        }
    #######################################################################
    #                           physical setup                            #
    #######################################################################
    # TODO: add marker picking
    # get up from the board
    robot._getT_w_e()
    current_pose = robot.getT_w_e()
    # Z
    #current_pose.translation[2] = current_pose.translation[2] + 0.03
    # Y
    #current_pose.translation[1] = current_pose.translation[1] + 0.03
    #moveL(args, robot, current_pose)
    # move to initial pose
    dmp.step(1/500)
    first_q = dmp.pos.reshape((6,))
    first_q = list(first_q)
    first_q.append(0.0)
    first_q.append(0.0)
    first_q = np.array(first_q)
    #pin.forwardKinematics(robot.model, robot.data, first_q)
    mtool = robot.getT_w_e(q_given=first_q)
    #mtool.translation[1] = mtool.translation[1] - 0.0035
    #mtool.translation[1] = mtool.translation[1] - 0.012
    mtool.translation[1] = mtool.translation[1] - 0.006
    if args.debug_prints:
        print("im at:", robot.getT_w_e())
        print("going to:", mtool)
    print('going to starting write position')
    # TODO: write a compliantMoveL - be careful that everything is in the body frame
    # since your jacobian is the body jacobian!!!
    #moveL(args, robot, mtool)
    if not args.board_wiping:
        compliantMoveL(args, robot, mtool)
    else:
        moveL(args, robot, mtool)

    #moveJ(args, robot, dmp.pos.reshape((6,)))
    controlLoop = partial(controlLoopWriting, dmp, tc, controller, robot)
    loop_manager = ControlLoopManager(robot, controlLoop, args, save_past_dict, log_item)
    # and now we can actually run
    log_dict, final_iteration = loop_manager.run()
    #loop_manager.stopHandler(None, None)
    mtool = robot.getT_w_e()
    print("move a bit back")
    if args.board_axis == 'z' or args.board_axis == 'y':
        mtool.translation[1] = mtool.translation[1] - 0.1
#    if args.board_axis == 'z':
#        mtool.translation[2] = mtool.translation[2] - 0.1
    compliantMoveL(args, robot, mtool)

    if args.visualize_manipulator:
        robot.killManipulatorVisualizer()

    #plotFromDict(log_dict, args)
    loop_manager.stopHandler(None, None)
    #robot.stopHandler(None, None)
    #robot.stopHandler(None, None)
    # plot results
    #plotFromDict(log_dict, args)
    # TODO: add some math to analyze path precision

    


