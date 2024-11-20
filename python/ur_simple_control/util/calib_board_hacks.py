# TODO: make prints prettier (remove \ in code, and also
# make it look good in the terminal)

import pinocchio as pin
import numpy as np
import time
import copy
from ur_simple_control.managers import RobotManager
from ur_simple_control.util.freedrive import freedrive
from ur_simple_control.clik.clik_point_to_point import moveL, moveUntilContact
# used to deal with freedrive's infinite while loop
import threading

"""
general
-----------
Estimate a plane by making multiple contacts with it. 
You need to start with a top left corner of it,
and you thus don't need to find an offset (you have to know it in advance).
TODO: test and make sure the above statement is in fact correct.
Thus the offset does not matter, we only need the angle,
i.e. the normal vector to the plane.
Returns R because that's what's needed to construct the hom transf. mat.
"""

"""
fitNormalVector
----------------
classic least squares fit.
there's also weighting to make new measurements more important,
beucase we change the orientation of the end-effector as we go. 
the change in orientation is done so that the end-effector hits the 
board at the angle of the board, and thus have consistent measurements.
"""
def fitNormalVector(positions):
    positions = np.array(positions)
    # non-weighted least squares as fallback (numerical properties i guess)
    n_non_weighted = np.linalg.lstsq(positions, np.ones(len(positions)), rcond=None)[0]
    n_non_weighted = n_non_weighted / np.linalg.norm(n_non_weighted)
    print("n_non_weighted", n_non_weighted)
    for p in positions:
        print("cdot none", p @ n_non_weighted)
    try:
        # strong
        W = np.diag(np.arange(1, len(positions) + 1))
        n_linearly_weighted = np.linalg.inv(positions.T @ W @ positions) @ positions.T @ W @ np.ones(len(positions))
        n_linearly_weighted = n_linearly_weighted / np.linalg.norm(n_linearly_weighted)
        print("n_linearly_weighed", n_linearly_weighted)
        print("if the following give you roughly the same number, it worked")
        for p in positions:
            print("cdot strong", p @ n_linearly_weighted)
        return n_linearly_weighted
    except np.linalg.LinAlgError:
        print("n_linearly_weighted is singular bruh")
    return n_non_weighted

"""
constructFrameFromNormalVector
----------------------------------
constuct a frame around the found normal vector
we just assume the x axis is parallel with the robot's x axis
this is of course completly arbitrary, so
TODO fix the fact that you just assume the x axis
or write down why you don't need it (i'm honestly not sure atm, but it is late)
"""
def constructFrameFromNormalVector(R_initial_estimate, n):
    z_new = n
    x_new = np.array([1.0, 0.0, 0.0])
    y_new = np.cross(x_new, z_new)
    # reshaping so that hstack works as expected
    R = np.hstack((x_new.reshape((3,1)), y_new.reshape((3,1))))
    R = np.hstack((R, z_new.reshape((3,1))))
    # now ensure all the signs are the signs that you want,
    # which we get from the initial estimate (which can not be that off)
    # NOTE this is potentially just an artifact of the previous solution which relied
    # on UR TCP readings which used rpy angles. but it ain't hurting nobody
    # so i'm leaving it.
    R = np.abs(R) * np.sign(R_initial_estimate)
    print('rot mat to new frame:')
    print(*R, sep=',\n')
    return R

"""
handleUserToHandleTCPPose
-----------------------------
1. tell the user what to do with prints, namely where to put the end-effector
  to both not break things and also actually succeed
2. start freedrive
3. use some keyboard input [Y/n] as a blocking call,
4. release the freedrive and then start doing the calibration process
"""
def handleUserToHandleTCPPose(robot):
    print("""
    Whatever code you ran wants you to calibrate the plane on which you will be doing
    your things. Put the end-effector at the top left corner SOMEWHAT ABOVE of the plane 
    where you'll be doing said things. \n
    Make sure the orientation is reasonably correct as that will be 
    used as the initial estimate of the orientation, 
    which is what you will get as an output from this calibration procedure.
    The end-effector will go down (it's TCP z pozitive direction) and touch the thing
    the number of times you specified (if you are not aware of this, check the
    arguments of the program you ran.\n 
    The robot will now enter freedrive mode so that you can manually put
    the end-effector where it's supposed to be.\n 
    When you did it, press 'Y', or press 'n' to exit.
    """)
    while True:
        answer = input("Ready to calibrate or no (no means exit program)? [Y/n]")
        if answer == 'n' or answer == 'N':
            print("""
    The whole program will exit. Change the argument to --no-calibrate or 
    change code that lead you here.
            """)
            exit()
        elif answer == 'y' or answer == 'Y':
            print("""
    The robot will now enter freedrive mode. Put the end-effector to the 
    top left corner of your plane and mind the orientation.
                    """)
            break
        else:
            print("Whatever you typed in is neither 'Y' nor 'n'. Give it to me straight cheif!")
            continue
    print("""
    Entering freedrive. 
    Put the end-effector to the top left corner of your plane and mind the orientation.
    Press Enter to stop freedrive.
    """)
    time.sleep(2)
    # it is necessary both to have freedrive as an infinite loop,
    # and to run it in another thread to stop it.
    # TODO: make this pretty (redefine freedrive or something, idc,
    # just don't have it here)
    def freedriveThreadFunction(robot, stop_event):
        robot.rtde_control.freedriveMode()
        while not stop_event.is_set():
            q = robot.rtde_receive.getActualQ()
            q.append(0.0)
            q.append(0.0)
            pin.forwardKinematics(robot.model, robot.data, np.array(q))
            print(robot.data.oMi[6])
            print("pin:", *robot.data.oMi[6].translation.round(4), \
                    *pin.rpy.matrixToRpy(robot.data.oMi[6].rotation).round(4))
            print("ur5:", *np.array(robot.rtde_receive.getActualTCPPose()).round(4))
            print("q:", *np.array(q).round(4))
            time.sleep(0.005)

    # Create a stop event
    stop_event = threading.Event()

    # Start freedrive in a separate thread
    freedrive_thread = threading.Thread(target=freedriveThreadFunction, args=(robot, stop_event,))
    freedrive_thread.start()

    input("Press Enter to stop...")
    stop_event.set()

    # join child thread (standard practise)
    freedrive_thread.join()

    while True:
        answer = input("""
    I am assuming you got the end-effector in the correct pose. \n
    Are you ready to start calibrating or not (no means exit)? [Y/n]
    """)
        if answer == 'n' or answer == 'N':
            print("The whole program will exit. Goodbye!")
            exit()
        elif answer == 'y' or answer == 'Y':
            print("Calibration about to start. Have your hand on the big red stop button!")
            time.sleep(2)
            break
        else:
            print("Whatever you typed in is neither 'Y' nor 'n'. Give it to me straight cheif!")
            continue
    robot.rtde_control.endFreedriveMode()

def getSpeedInDirectionOfN(R, board_axis):
    # TODO: make this general
    # TODO: FIX WHAT YOU MEAN BY Y AND Z
    ############ 
    # this is for z
    if board_axis == 'y':
        z_of_rot = R[:,2]
        speed = np.array([z_of_rot[0], z_of_rot[1], z_of_rot[2], 0, 0, 0])
    ############ 
    # this is for y
    elif board_axis == 'z':
        y_of_rot = R[:,1]
        speed = np.array([y_of_rot[0], y_of_rot[1], y_of_rot[2], 0, 0, 0])
    else:
        print("you passed", board_axis, ", but it has to be 'z' or 'y'")
        exit()
    # make speed small no matter what
    speed = speed / np.linalg.norm(speed)
    # nice 'n' slow
    # TODO: remove magic number
    speed = speed / 40
    #speed[2] = -1 * speed[2]
    speed = -1 * speed
    print("going in", speed, "direction")
    return speed

# TODO
# be internally consistent! this will also make this code modular,
# and robot-agnostic!
# drop all stupid ur code, and use your own implementation
# of both movel and speedj.
# --> then you don't even need to use the stupid rpy,
# but instead rock only rotation matrices the way it should be.
# TODO: replace all moveUntilContacts with your own implementation
# and of course replace the stupid speeds
# TODO move width and height, rock just args
def calibratePlane(args, robot, plane_width, plane_height, n_tests):
    # i don't care which speed slider you have,
    # because 0.4 is the only reasonable one here
#    old_speed_slider = robot.speed_slider
#    robot.setSpeedSlider(0.7)
    handleUserToHandleTCPPose(robot)
    q_init = copy.deepcopy(robot.getQ())
    # GET TCP
    Mtool = robot.getT_w_e()

    init_pose = copy.deepcopy(Mtool)
    new_pose = copy.deepcopy(init_pose)

    # you just did it dawg.
    # i mean i know it's in the robot
    # TODO: don't manage forward kinematics yourself,
    # just create a RobotManager.step() function, update everything there
    #q = robot.getQ()
    #pin.forwardKinematics(robot.model, robot.data, q)
    # this apsolutely has to be deepcopied aka copy-constructed
    #R_initial_estimate = copy.deepcopy(robot.data.oMi[robot.JOINT_ID].rotation)
    R_initial_estimate = copy.deepcopy(Mtool.rotation)
    print("R_initial_estimate", R_initial_estimate)
    R = copy.deepcopy(R_initial_estimate)

    # go in the TCP z direction of the end-effector
    # our goal is to align that with board z
    #speed = getSpeedInDirectionOfN(R_initial_estimate, args.board_axis)
    speed = np.zeros(6)
    speed[2] = 0.02
    print("speed", speed)
    # get q, do forward kinematics, get current TCP R 

    positions = []
    for i in range(n_tests):
        time.sleep(0.01)
        print("iteration number:", i)
        #robot.rtde_control.moveUntilContact(speed)
        moveUntilContact(args, robot, speed)
        # TODO: replace forwardkinematics call with robot.step()
        q = robot.getQ()
        pin.forwardKinematics(robot.model, robot.data, np.array(q))
        print("pin:", *robot.data.oMi[robot.JOINT_ID].translation.round(4), \
                *pin.rpy.matrixToRpy(robot.data.oMi[robot.JOINT_ID].rotation).round(4))
        print("ur5:", *np.array(robot.rtde_receive.getActualTCPPose()).round(4))

        positions.append(copy.deepcopy(robot.data.oMi[robot.JOINT_ID].translation))
        if i < n_tests -1:
            current_pose = robot.getT_w_e()
            new_pose = copy.deepcopy(current_pose)
            # go back up (assuming top-left is highest incline)
            # TODO: make this assumption an argument, or print it at least
            # THIS IS Z
            if args.board_axis == 'z':
                new_pose.translation[2] = init_pose.translation[2]
            # THIS IS Y
            if args.board_axis == 'y':
                new_pose.translation[1] = init_pose.translation[1]
            moveL(args, robot, new_pose)
            q = robot.getQ()
            pin.forwardKinematics(robot.model, robot.data, np.array(q))
            # TODO: make this not base-orientation dependent,
            # this is also an ugly ugly hack
            new_pose.translation[0] = init_pose.translation[0] + np.random.random() * plane_width
            # THIS IS Z
            if args.board_axis == 'z':
                new_pose.translation[1] = init_pose.translation[1] - np.random.random() * plane_height
            # THIS IS Y
            if args.board_axis == 'y':
                new_pose.translation[2] = init_pose.translation[2] - np.random.random() * plane_height
            moveL(args, robot, new_pose)
            # fix orientation
            new_pose.rotation = R
            moveL(args, robot, new_pose)
        # skip the first one
        if i > 2:
            n = fitNormalVector(positions)
            R = constructFrameFromNormalVector(R_initial_estimate, n)
            #speed = getSpeedInDirectionOfN(R, args.board_axis)
            #speed = getSpeedInDirectionOfN(R_initial_estimate, args.board_axis)
            speed = np.zeros(6)
            speed[2] = 0.02

    print("finished estimating R")

    current_pose = robot.getT_w_e()
    new_pose = copy.deepcopy(current_pose)
    # go back up
    # Z
    if args.board_axis == 'z':
        new_pose.translation[2] = init_pose.translation[2]
    # Y
    if args.board_axis == 'y':
        new_pose.translation[1] = init_pose.translation[1]
    moveL(args, robot, new_pose)
    # go back to the same spot
    new_pose.translation[0] = init_pose.translation[0]
    new_pose.translation[1] = init_pose.translation[1]
    new_pose.translation[2] = init_pose.translation[2]
    # but in new orientation
    new_pose.rotation = R
    moveL(args, robot, new_pose)
    
    # TODO there's certainly no need for all of these moves bro
    # --> now you're ready to measure the translation vector correctly
    # for this we want to go directly into the board
    print("i'll estimate the translation vector now")
    #speed = getSpeedInDirectionOfN(R_initial_estimate, args.board_axis)
    speed = np.zeros(6)
    speed[2] = 0.02

    moveUntilContact(args, robot, speed)

    q = robot.getQ()
    pin.forwardKinematics(robot.model, robot.data, np.array(q))
    translation = copy.deepcopy(robot.data.oMi[robot.JOINT_ID].translation)
    print("got translation vector, it's:", translation)

    # TODO: get rid of all movels, just your own stuff,
    # or at least shove them avait to the RobotManager
    # and now go back up
    current_pose = robot.getT_w_e()
    new_pose = copy.deepcopy(current_pose)
    new_pose.translation[2] = init_pose.translation[2]
    moveL(args, robot, new_pose)
    q = robot.getQ()
    init_q = copy.deepcopy(q)
    print("went back up, saved this q as initial q")
    
    # put the speed slider back to its previous value
#    robot.setSpeedSlider(old_speed_slider)
    print('also, the translation vector is:', translation)
    return R, translation, q_init

# TODO: remove once you know shit works (you might be importing incorectly)
#if __name__ == "__main__":
#    robot = RobotManager()
#    # TODO make this an argument
#    n_tests = 10
#    # TODO: 
#    # - tell the user what to do with prints, namely where to put the end-effector
#    #   to both not break things and also actually succeed
#    # - start freedrive
#    # - use some keyboard input [Y/n] as a blocking call,
#    #   release the freedrive and then start doing the calibration process
#    calibratePlane(robot, n_tests)
