# your entry point is a 
# np.array of shape (N_POINTS, 2)
# this files results in a timed joint path

# STEP 1: map the pixels to a 3D plane
# STEP 2: clik that path
# STEP 3: put result into csv (but also save it in a convenient class/array here)

import numpy as np
import pinocchio as pin
import copy
from ur_simple_control.managers import ControlLoopManager

#######################################################################
#                    map the pixels to a 3D plane                     #
#######################################################################
def map2DPathTo3DPlane(path_2D, width, height):
    """
    map2DPathTo3DPlane
    --------------------
    TODO: THINK AND FINALIZE THE FRAME
    TODO: WRITE PRINT ABOUT THE FRAME TO THE USER
    assumptions:
    - origin in top-left corner (natual for western latin script writing)
    - x goes right (from TCP)
    - z will go away from the board
    - y just completes the right-hand frame
    TODO: RIGHT NOW we don't have a right-handed frame lmao, change that where it should be
    NOTE: this function as well be in the util or drawing file, but whatever for now, it will be done
          once it will actually be needed elsewhere
    Returns a 3D path appropriately scaled, and placed into the first quadrant
    of the x-y plane of the body-frame (TODO: what is the body frame if we're general?)
    """
    z = np.zeros((len(path_2D),1))
    path_3D = np.hstack((path_2D,z))
    # scale the path to m
    path_3D[:,0] = path_3D[:,0] * width
    path_3D[:,1] = path_3D[:,1] * height
    # in the new coordinate system we're going in the -y direction
    # TODO this is a demo specific hack, 
    # make it general for a future release
    path_3D[:,1] = -1 * path_3D[:,1] + height
    return path_3D


def clikCartesianPathIntoJointPath(path, args, robot, \
        clikController, q_init, R, p):
    """
    clikCartesianPathIntoJointPath
    ------------------------------
    functionality
    ------------
    Follows a provided Cartesian path,
    creates a joint space trajectory for it.

    return
    ------
    - joint_space_trajectory to follow the given path.

    arguments
    ----------
    - path:
      --> cartesian path given in task frame
    TODO: write asserts for these arguments
    - args:
      --> all the magic numbers used here better be here
    - clikController:
      --> clik controller you're using
    - robot:
      --> RobotManager instance (needed for forward kinematics etc)
    TODO: maybe it's better design to expect path in body frame idk
          the good thing here is you're forced to think about frames.
    TODO: make this a kwarg with a neural transform as default.
    - transf_body_task_frame: 
      --> A transformation from the body frame to task frame.
      --> It is assumed that the path was provided in the task frame
          because 9/10 times that's more convenient,
          and you can just pass a neural transform if you're not using it.
    - q_init:
      --> starting point. 
      --> you can movej to it before executing the trajectory,
         so this makes perfect sense to ensure correctness
    """

    transf_body_to_task_frame = pin.SE3(R, p)
    q = copy.deepcopy(q_init)

    for i in range(len(path)):
        path[i] = transf_body_to_task_frame.act(path[i])
    # TODO remove print, write a test for this instead
    if args.debug_prints:
        print(path)

    # TODO: finish this
    # - pass clik alg as argument
    # - remove magic numbers
    # - give output in the right format
    # skip inital pos tho
    #q = np.array([-2.256,-1.408,0.955,-1.721,-1.405,-0.31, 0.0, 0.0])
    #q = np.array([-2.014, -1.469, 1.248, -1.97, -1.366, -0.327, 0.0, 0.0])
    # this is just init_q right now
    # TODO: make this a flag or something for readability's sake
    n_iter = args.max_init_clik_iterations
    # we don't know how many there will be, so a linked list is 
    # clearly the best data structure here (instert is o(1) still,
    # and we aren't pressed on time when turning it into an array later)
    qs = []
    for goal in path:
        Mgoal = pin.SE3(R, goal)
        for i in range(n_iter):
            # TODO maybe hide pin call with RobotManager call
            pin.forwardKinematics(robot.model, robot.data, q)
            SEerror = robot.data.oMi[robot.JOINT_ID].actInv(Mgoal)
            err_vector = pin.log6(SEerror).vector 
            if np.linalg.norm(err_vector) < args.clik_goal_error:
                if not n_iter == args.max_init_clik_iterations:
                    if args.debug_prints:
                        print("converged in", i)
                    break
            J = pin.computeJointJacobian(robot.model, robot.data, q, robot.JOINT_ID)
            qd = clikController(J, err_vector)
            # we're integrating this fully offline of course
            q = pin.integrate(robot.model, q, qd * robot.dt)
            if (not n_iter == args.max_init_clik_iterations) and (i % 10 == 0):
                qs.append(q[:6])

        # just skipping the first run with one stone
        if n_iter == args.max_init_clik_iterations:
            n_iter = args.max_running_clik_iterations
        else:
            if i == args.max_running_clik_iterations - 1:
                print("DID NOT CONVERGE -- exiting")
                # nothing is moving 
                # and i'm not even using a manager here
                # so no need, right?
                #ControlLoopManager.stopHandler(None, None, None)
                exit()

    ##############################################
    #  save the obtained joint-space trajectory  #
    ##############################################
    qs = np.array(qs)
    # we're putting a dmp over this so we already have the timing ready
    # TODO: make this general, you don't want to depend on other random
    # arguments (make this one traj_time, then put tau0 = traj_time there
    t = np.linspace(0, args.tau0, len(qs)).reshape((len(qs),1))
    joint_trajectory = np.hstack((t, qs))
    # TODO handle saving more consistently/intentionally
    # (although this definitely works right now and isn't bad, just mid)
    # os.makedir -p a data dir and save there, this is ugly
    np.savetxt("./joint_trajectory.csv", joint_trajectory, delimiter=',', fmt='%.5f')
    return joint_trajectory



