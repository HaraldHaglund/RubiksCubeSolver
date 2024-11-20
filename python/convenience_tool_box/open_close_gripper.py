from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from rtde_io import RTDEIOInterface
from ur_simple_control.util.robotiq_gripper import RobotiqGripper
import pinocchio as pin
import numpy as np
import os
import time
import signal
import matplotlib.pyplot as plt


def handler(signum, frame):
    gripper.move(255,255,255)
    time.sleep(2)
    exit()

signal.signal(signal.SIGINT, handler)

gripper = RobotiqGripper()
gripper.connect("192.168.1.102", 63352)
#gripper.connect("192.168.1.3", 63352)
time.sleep(6)
gripper.activate()
#time.sleep(3)
gripper.move(0,100,100)
time.sleep(4)
gripper.move(255,255,255)
#time.sleep(300)
input("press Enter to close and quit")
handler(None, None)
