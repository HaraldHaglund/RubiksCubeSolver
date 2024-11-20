import numpy as np
import time
import argparse
from functools import partial
from ur_simple_control.managers import ControlLoopManager, RobotManager
# TODO merge the 2 clik files
from ur_simple_control.clik.clik_point_to_point import getClikController, controlLoopClik, moveL, compliantMoveL
# TODO write this in managers and automate name generation
from ur_simple_control.util.logging_utils import saveLog
from  ur_simple_control.basics.basics import moveJ
from pinocchio import SE3
import time
import os

import roboticstoolbox as rtb
from spatialmath import SE3 as SE

# arp-scan 192.168.1.1/24
# För simulering
# python3 robot.py --pinocchio-only --visualize-manipulator 

# För live
# python3 robot.py --visualize-manipulator --robot-ip=192.168.1.103 --gripper=onrobot
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
            help="gripper you're using (no gripper is the default)", \
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
    parser.add_argument('--tikhonov-damp', type=float, \
            help="damping scalar in tikhonov regularization", default=1e-3)
    parser.add_argument('--minimum-detectable-force-norm', type=float, \
            help="we need to disregard noise to converge despite filtering. \
                  a quick fix is to zero all forces of norm below this argument threshold.",
                 default=3.0)
    parser.add_argument("--start-from-current-pose", action=argparse.BooleanOptionalAction, \
            help="if connected to the robot, read the current pose and set it as the initial pose for the robot. \
                 very useful and convenient when running simulation before running on real", \
                         default=False)
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


#--------------------------------------------------------
#-------- helper functions and basic functions ----------
#--------------------------------------------------------

# rotate layer i steps (i*90 degrees, i*pi/2 radiants)
# i can be negative
def rotate_layer(i):
    jpos = get_joint_pos()
    jpos[5]+=i*np.pi/2
    _moveJ(jpos)
    return

# gripper at halfway down the cube
def move_to_pick(rot):
    _moveL(rot,pick_pos)
    return

# gripper at upper layer, ready to rotate
def move_to_upper_layer():
    _moveL(down_rot,upper_layer_pos)
    return

# reset joints to home position
def moveJ_home():
    # values from jogging robot
    # degrees converted to radians
    _moveJ(np.array([82.3,-76.4,92.3,-105.7,-89.5,-6,0,0])*np.pi/180)
    return

def _moveJ(array_of_eight): 
    moveJfast(args,robot,array_of_eight)
    return
# our moveL with tool offset
def _moveL(rot,pos): 
    moveL(args,robot,SE3(rot,pos)* SE3(np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]), np.array([0, 0, -gripper_offset_z])))
    return

def move_z(i):
    T = get_cart_pos()
    _moveL(T.rotation,T.translation+np.array([0,0,i]))
    return

def moveJControlLoop(q_desired, robot, i, past_data):
    """
    PID control for joint space point-to-point motion with approximated joint velocities.
    """

    # ================================
    # Initialization
    # ================================
    breakFlag = False
    save_past_dict = {}
    log_item = {}

    # ================================
    # Current Joint Positions
    # ================================
    q = robot.getQ()[:6]  # Get current joint positions (first 6 joints)
    # Ensure q is a NumPy array of dtype float64
    q = np.ascontiguousarray(q, dtype=np.float64).flatten()

    # ================================
    # Retrieve Previous States
    # ================================
    q_prev = past_data['q_prev'][-1]    # Previous joint positions
    e_prev = past_data['e_prev'][-1]    # Previous position error
    ed_prev = past_data['ed_prev'][-1]  # Previous derivative of error

    # ================================
    # Approximate Actual Joint Velocities
    # ================================
    # Using finite differences
    qd_actual = (q - q_prev) / robot.dt

    # ================================
    # Compute Position Error
    # ================================
    q_desired = np.ascontiguousarray(q_desired[:6], dtype=np.float64).flatten()
    q_error = q_desired - q  # Position error

    # ================================
    # Compute Derivative of Position Error
    # ================================
    # Approximate derivative of error using finite differences
    e_d = (q_error - e_prev) / robot.dt
    e_d = np.ascontiguousarray(e_d, dtype=np.float64).flatten()
    ed_prev = np.ascontiguousarray(ed_prev, dtype=np.float64).flatten()

    # Apply low-pass filter to derivative of error
    alpha = 0.5  # Filter coefficient (adjust as needed)
    ed_filtered = alpha * e_d + (1 - alpha) * ed_prev

    # ================================
    # Check for Convergence
    # ================================
    if np.linalg.norm(q_error) < 1e-3 and np.linalg.norm(qd_actual) < 1e-3:
        breakFlag = True  # Convergence achieved

    # ================================
    # Update Integral of Error
    # ================================
    integral_error = past_data['integral_error'][-1]
    integral_error = np.ascontiguousarray(integral_error, dtype=np.float64).flatten()
    integral_error += q_error * robot.dt  # Accumulate error over time

    # Anti-windup: Limit integral error to prevent excessive accumulation
    max_integral = 10
    integral_error = np.clip(integral_error, -max_integral, max_integral)

    # ================================
    # Save Current States for Next Iteration
    # ================================
    save_past_dict['integral_error'] = [integral_error]  # Save updated integral error
    save_past_dict['q_prev'] = [q]                       # Save current joint positions
    save_past_dict['e_prev'] = [q_error]                 # Save current position error
    save_past_dict['ed_prev'] = [ed_filtered]                    # Save current derivative of error

    # ================================
    # Control Gains
    # ================================
    Kp = 7.0  # Proportional gain
    Ki = 0.0  # Integral gain
    Kd = 3  # Derivative gain

    # ================================
    # Compute Control Input (Joint Velocities)
    # ================================
    qd = Kp * q_error + Ki * integral_error + Kd * ed_filtered
    qd[5]=qd[5]*10
    # Ensure qd is a NumPy array of dtype float64
    qd = np.ascontiguousarray(qd, dtype=np.float64).flatten()

    # ================================
    # Send Joint Velocities to the Robot
    # ================================
    robot.sendQd(qd)

    return breakFlag, save_past_dict, log_item

def moveJfast(args, robot, q_desired):
    assert isinstance(q_desired, np.ndarray)
    controlLoop = partial(moveJControlLoop, q_desired, robot)

    # ================================
    # Initialization
    # ================================
    # Get initial joint positions
    initial_q = robot.getQ()[:6]
    initial_q = np.ascontiguousarray(initial_q, dtype=np.float64).flatten()

    # Initialize integral error to zeros
    initial_integral_error = np.zeros_like(q_desired[:6], dtype=np.float64)

    # Initialize past data for control loop
    # Note: e_prev and ed_prev are initialized with initial_q,
    # which represents the initial state but may need adjustment
    save_past_dict = {
        'integral_error': [initial_integral_error],
        'q_prev': [initial_q],
        'e_prev': [initial_q],   # Initial position error (may need to be q_desired - initial_q)
        'ed_prev': [initial_q]   # Initial derivative of error (may need to be zeros)
    }

    log_item = {}  # Initialize log item (if logging is needed)

    # ================================
    # Create and Run Control Loop Manager
    # ================================
    loop_manager = ControlLoopManager(robot, controlLoop, args, save_past_dict, log_item)
    log_dict, final_iteration = loop_manager.run()

    # ================================
    # Debug Printing
    # ================================
    if args.debug_prints:
        print("MoveJ done: convergence achieved, reached destination!")

def _compute_joint_angles(pos, rot):
    return compute_joint_angles(pos[0],pos[1],pos[2],rot)

def compute_joint_angles(x, y, z, rotation_matrix):
    """
    Computes joint angles using the robot model's inverse kinematics.

    Args:
        x, y, z (float): Desired Cartesian coordinates.
        rotation_matrix (np.ndarray): Desired orientation as a 3x3 rotation matrix.

    Returns:
        np.ndarray: Joint angles solution.
    """
    T = SE.Rt(rotation_matrix,[x,y,z])* SE.Rt(np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]), np.array([0, 0, -gripper_offset_z]))

    # Initial guess for joint angles
    q_initial = get_joint_pos()[:6]

    # Solve inverse kinematics
    solution = inv_robot.ikine_LM(T, q0=q_initial)

    if solution.success:
        return solution.q
    else:
        raise ValueError("Inverse kinematics did not converge")

def get_joint_pos():
    return robot.getQ()

def get_cart_pos():
    return robot.getT_w_e()*SE3(np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]), np.array([0, 0, gripper_offset_z]))

def _define_robot_model():
    # Define joint limits (assuming [-pi, pi] for all joints)
    qlim = np.array([[-np.pi, np.pi]] * 6)

    # Define the robot using DH parameters
    L = [
        rtb.RevoluteDH(d=0.1625, a=0, alpha=np.pi/2, qlim=qlim[0]),
        rtb.RevoluteDH(d=0, a=-0.425, alpha=0, qlim=qlim[1]),
        rtb.RevoluteDH(d=0, a=-0.3922, alpha=0, qlim=qlim[2]),
        rtb.RevoluteDH(d=0.1333, a=0, alpha=np.pi/2, qlim=qlim[3]),
        rtb.RevoluteDH(d=0.0997, a=0, alpha=-np.pi/2, qlim=qlim[4]),
        rtb.RevoluteDH(d=0.0996, a=0, alpha=0, qlim=qlim[5])
    ]

    # Combine the links into a SerialLink robot model
    robot_model = rtb.SerialLink(L, name='CustomRobot')
    return robot_model
    
#closeGripper
def grab():

    _moveJ(get_joint_pos()[:6])
    #time.sleep(5)
    robot.closeGripper()
    _moveJ(get_joint_pos()[:6])
    return
#opengripper
def drop():
    _moveJ(get_joint_pos()[:6])
    #time.sleep(5)
    robot.openGripper()
    _moveJ(get_joint_pos()[:6])
    return


#--------------------------------------------------------
#---------------- bigger **cooler** functions -----------
#--------------------------------------------------------
def read_cube():
    with open(os.path.join(os.path.dirname(__file__), "solution.txt"), 'r') as f:
        l = [line.rstrip('\n') for line in f]
        #print(f'List gotten from file: {l}')
    return l
def solve(moves):
    for move in moves:
        if move == 'F':
            print("Doing F")
            F()
        elif move == "F'":
            print("Doing F'")
            F_prime()
        elif move == 'F2':
            print("Doing F2")
            F2()
        elif move == 'U':
            print("Doing U")
            U()
        elif move == "U'":
            print("Doing U'")
            U_prime()
        elif move == 'U2':
            print("Doing U2")
            U2()
        elif move == 'R':
            print("Doing R")
            R()
        elif move == "R'":
            print("Doing R'")
            R_prime()
        elif move == 'R2':
            print("Doing R2")
            R2()
        elif move == 'L':
            print("Doing L")
            L()
        elif move == "L'":
            print("Doing L'")
            L_prime()
        elif move == 'L2':
            print("Doing L2")
            L2()
        elif move == 'D':
            print("Doing D")
            D()
        elif move == "D'":
            print("Doing D'")
            D_prime()
        elif move == 'D2':
            print("Doing D2")
            D2()
        elif move == 'B':
            print("Doing B")
            B()
        elif move == "B'":
            print("Doing B'")
            B_prime()
        elif move == 'B2':
            print("Doing B2")
            B2()
        else:
            print(f"Move {move} not implemented")
    return

def B(): 
    _B(1)
    return
def B2():
    _B(2)
    return
def B_prime():
    _B(-1)
    return

def _B(i):
    # Note: Must start at home!

    #rotate cube and drop
    drop()
    moveJ_home()
    
    ang=get_joint_pos()
    ang[5]-=np.pi/2
    _moveJ(ang)
    
    #move_to_pick(down_rot)
    move_z(pick_pos[2]-get_cart_pos().translation[2]) #Gå ner längre!
    grab()
    move_z(0.1)
    
    moveJ_home()
    
    ang=_compute_joint_angles(home_pos+np.array([-0.005,0.017,0]),forward_rot)
    ang[5]-=np.pi/2
    _moveJ(ang)
    
    move_z(drop_pos[2]-get_cart_pos().translation[2])
    drop()
    move_z(0.1)
    
    moveJ_home()
    
    move_to_upper_layer()
    
    grab()
    rotate_layer(i)
    drop()
    if(i==2):
        move_z(0.05)
        rotate_layer(-1)
    
    move_z(pick_pos[2]-get_cart_pos().translation[2])
    grab()
    
    move_z(0.1)
    
    moveJ_home()
    
    ang=_compute_joint_angles(home_pos+np.array([-0.005,0.017,0]),forward_rot)
    ang[5]+=np.pi/2
    _moveJ(ang)
    move_z(drop_pos[2]-get_cart_pos().translation[2])
    
    drop()
    
    move_z(0.1)
    moveJ_home()

def F():  
    _F(1)
    return
def F2():
    _F(2)
    return
def F_prime():
    _F(-1)
    return

def _F(i):
      # Note: Must start at home!

    #"initialize"
    drop()
    moveJ_home()
    
    # turn 90 degrees EE
    ang=get_joint_pos()
    ang[5]-=np.pi/2
    _moveJ(ang)
    
    #pick up
    move_z(pick_pos[2]-get_cart_pos().translation[2]) #BRA höjd -0.13!
    grab()
    move_z(0.1)
    moveJ_home()
    
    # turn arm to forward
    ang=_compute_joint_angles(home_pos+np.array([0,0.017,0]),forward_rot)
    ang[5]+=np.pi/2
    _moveJ(ang)
    
    #move down, drop, move up
    move_z(pick_pos[2]-get_cart_pos().translation[2])
    drop()
    move_z(0.1)
    
    #reset arm
    moveJ_home()
    
    move_to_upper_layer()
    
    #rotate layer
    grab()
    rotate_layer(i)
    drop()
    
    # le squeeze :)
    move_z(pick_pos[2]-get_cart_pos().translation[2])
    grab()
    drop()
    
    # move up and home
    move_z(0.1)
    moveJ_home()
    
    # turn arm to forward
    ang=_compute_joint_angles(home_pos+np.array([0,0.01,0]),forward_rot)
    ang[5]-=np.pi/2
    _moveJ(ang)
    
    #move down
    move_z(pick_pos[2]-get_cart_pos().translation[2])
    
    grab()
    
    #move up, move home
    move_z(0.1)
    moveJ_home()
    
    #rotate EE back
    ang=_compute_joint_angles(home_pos+np.array([0,-0.017,0]),down_rot)
    ang[5]+=np.pi/2
    _moveJ(ang)
    
    #move to drop, drop
    move_z(drop_pos[2]-get_cart_pos().translation[2])
    drop()
    
    #move up and home
    move_z(0.1)
    moveJ_home()

def U(): 
    _U(1)
    return
def U2():
    _U(2)
    return
def U_prime():
    _U(-1)
    return

def _U(i):
    # Note: Must start at home!

    #"initialize"
    drop()
    moveJ_home()

    # rotate upper layer
    move_to_upper_layer()
    grab()
    rotate_layer(i)
    drop()
    
    #squeeze :)
    move_z(pick_pos[2]-get_cart_pos().translation[2])
    grab()
    drop()
    
    #move up
    move_z(0.03)
    #move home
    moveJ_home()

def D(): 
    _D(1)
    return
def D2():
    _D(2)
    return
def D_prime():
    _D(-1)
    return

def _D(i):
    # Note: Must start at home!

    #"initialize"
    drop()
    moveJ_home()
    
    #rotate arm
    _moveJ(_compute_joint_angles(pick_up_left_side,right_rot))
    #move down
    move_z(pick_pos[2]-get_cart_pos().translation[2])
    #grab
    grab()
    #move up
    move_z(0.1)
    
    #rotate EE 180 degrees
    ang=get_joint_pos()
    ang[5]+=np.pi
    _moveJ(ang)
    
    #move down, drop, move up and home
    move_z(drop_pos[2]-0.007-get_cart_pos().translation[2]) 
    drop()
    move_z(0.1)
    moveJ_home()
    
    #rotate
    move_to_upper_layer()
    grab()
    rotate_layer(i)
    drop()
      
    #squeeze
    move_z(pick_pos[2]-get_cart_pos().translation[2])
    grab()
    drop()
    
    #move up and home
    move_z(0.1)
    moveJ_home()
    
    #rotate arm
    _moveJ(_compute_joint_angles(pick_up_left_side,right_rot))
    #move down
    move_z(pick_pos[2]-get_cart_pos().translation[2])
    #grab and move up
    grab()
    move_z(0.1)
    #rotate EE 180 degrees
    ang=get_joint_pos()
    ang[5]+=np.pi
    _moveJ(ang)
    
    #move down, drop, move up and home
    move_z(drop_pos[2]-0.007-get_cart_pos().translation[2])
    drop()
    move_z(0.1)
    moveJ_home() 
    
def R():
    _R(1)
    return
def R2():
    _R(2)
    return
def R_prime():
    _R(-1)
    return

def _R(i):
    # Note: Must start at home!
    #"initialize"
    drop()
    moveJ_home()
    
    # grab and move home
    move_to_pick(down_rot)
    grab()
    move_z(0.1)
    moveJ_home()
    
    #move arm to forward rot
    ang=_compute_joint_angles(home_pos+np.array([0,0.01,0]),forward_rot)
    ang[5]-=np.pi/2
    _moveJ(ang)
    
    #_moveL(forward_rot,home_pos)
    move_z(pick_pos[2]-get_cart_pos().translation[2])
    #_moveL(forward_rot,drop_pos)
    
    drop()
    print("I dropped the cube")
    #_moveL(forward_rot,home_pos)
    move_z(0.1)
    moveJ_home()
    
    #rotate layer
    move_to_upper_layer()
    grab()
    rotate_layer(i)
    drop()
    
    #squeeze :)
    move_z(pick_pos[2]-get_cart_pos().translation[2])
    grab()
    drop()
    
    move_z(0.1)
    
    #gripper is now 90 degrees off from home
    #need to go up without rotating to avoid hitting cube?
    #_moveL(...)
    moveJ_home()
    #_moveJ(_compute_joint_angles(home_pos,forward_rot)) # ska vara forward igentligen
    ang=_compute_joint_angles(home_pos+np.array([0,0.01,0]),forward_rot)
    ang[5]-=np.pi/2
    _moveJ(ang) # ska vara forward igentligen
    move_z(pick_pos[2]-get_cart_pos().translation[2])
    grab()
    move_z(0.1)
    moveJ_home()
    _moveL(down_rot,drop_pos+np.array([0.015,-0.003,0]))
    drop()
    
    move_z(0.1)
    
    moveJ_home()

def L():
    _L(1)
    return
def L2():
    _L(2)
    return
def L_prime():
    _L(-1)
    return

def _L(i):
    # Note: Must start at home!

    #rotate cube and drop
    drop()
    moveJ_home()
    move_to_pick(down_rot)
    
    grab()
    move_z(0.1)
    moveJ_home()
    #_moveJ(_compute_joint_angles(home_pos,forward_rot))
    ang=_compute_joint_angles(home_pos+np.array([-0.005,0.017,0]),forward_rot)
    ang[5]+=np.pi/2
    _moveJ(ang)
    #_moveL(forward_rot,home_pos)
    move_z(drop_pos[2]-get_cart_pos().translation[2])
    #_moveL(forward_rot,drop_pos)
    
    drop()
    print("I dropped the cube")
    #_moveL(forward_rot,home_pos)
    move_z(0.1)
    moveJ_home()
    
    #rotate layer
    move_to_upper_layer()
    grab()
    rotate_layer(i)
    drop()
    
    move_z(pick_pos[2]-get_cart_pos().translation[2])
    grab()
    drop()
    
    move_z(0.1)
    
    #gripper is now 90 degrees off from home
    #need to go up without rotating to avoid hitting cube?
    #_moveL(...)
    moveJ_home()
    #_moveJ(_compute_joint_angles(home_pos,forward_rot)) # ska vara forward igentligen
    ang=_compute_joint_angles(home_pos+np.array([0,0.01,0]),forward_rot)
    ang[5]+=np.pi/2
    _moveJ(ang) # ska vara forward igentligen
    move_z(pick_pos[2]-get_cart_pos().translation[2])
    grab()
    move_z(0.1)
    moveJ_home()
    _moveL(down_rot,drop_pos+np.array([-0.015,0,0]))
    drop()
    
    move_z(0.1)
    
    moveJ_home()
    




if __name__ == "__main__": 
    #pillabli (pilla inte)
    args = get_args()
    robot = RobotManager(args)
    inv_robot = _define_robot_model()
    time.sleep(1) # ZZZzzz
    
    #rotations
    down_rot = np.array([[-1,0,0],[0,1,0],[0,0,-1]])  #Down
    forward_rot = np.array([[1,0,0],[0,0,-1],[0,1,0]])  #gripper pointing towards us
    right_rot = np.array([[0,0,1],[0,1,0],[-1,0,0]]) 

    #positions
    gripper_offset_z = 0.143
    home_pos = np.array([0.0557, -0.5887, 0.20])
    pick_pos = np.array([0.0557, -0.5887, 0.092])
    upper_layer_pos = np.array([0.0557, -0.5887, 0.11]) 
    drop_pos = np.array([0.0557, -0.5887,0.11]) 
    pick_up_left_side = np.array([0.045,-0.59, 0.2])
    # variables for positions and stuff
    

    # -------------------"program" starts here --------------------------------------
  
    moves = ["U2"]
    #solve(moves)
    solve(read_cube())
    
    #_moveJ(_compute_joint_angles(pick_up_left_side,right_rot))
    
    
    
    
    
    # pillabli (pilla inte)
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

