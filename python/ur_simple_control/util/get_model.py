"""
possible improvement: 
    get the calibration file of the robot without ros
    and then put it here.
    these magic numbers are not a good look.
"""
import pinocchio as pin
import numpy as np
import sys
import os
from importlib.resources import files

# can't get the urdf reading with these functions to save my life, idk what or why

#############################################################
# PACKAGE_DIR IS THE WHOLE UR_SIMPLE_CONTROL FOLDER (cos that's accessible from anywhere it's installed)
# PACKAGE:// IS WHAT'S BEING REPLACED WITH THE PACKAGE_DIR ARGUMENT IN THE URDF.
# YOU GIVE ABSOLUTE PATH TO THE URDF THO.
#############################################################

"""
loads what needs to be loaded.
calibration for the particular robot was extracted from the yml
obtained from ros and appears here as magic numbers.
i have no idea how to extract calibration data without ros
and i have no plans to do so.
aligning what UR thinks is the world frame
and what we think is the world frame is not really necessary,
but it does aleviate some brain capacity while debugging.
having that said, supposedly there is a big missalignment (few cm)
between the actual robot and the non-calibrated model.
NOTE: this should be fixed for a proper release
"""
def get_model():
    
    urdf_path_relative = files('ur_simple_control.robot_descriptions.urdf').joinpath('ur5e_with_robotiq_hande_FIXED_PATHS.urdf')
    urdf_path_absolute = os.path.abspath(urdf_path_relative)
    mesh_dir = files('ur_simple_control')
    mesh_dir_absolute = os.path.abspath(mesh_dir)

    shoulder_trans = np.array([0, 0, 0.1625134425523304])
    shoulder_rpy = np.array([-0, 0, 5.315711138647629e-08])
    shoulder_se3 = pin.SE3(pin.rpy.rpyToMatrix(shoulder_rpy),shoulder_trans)

    upper_arm_trans = np.array([0.000300915150907851, 0, 0])
    upper_arm_rpy = np.array([1.571659987714477, 0, 1.155342090832558e-06])
    upper_arm_se3 = pin.SE3(pin.rpy.rpyToMatrix(upper_arm_rpy),upper_arm_trans)

    forearm_trans = np.array([-0.4249536100418752, 0, 0])
    forearm_rpy = np.array([3.140858652067472, 3.141065383898231, 3.141581851193229])
    forearm_se3 = pin.SE3(pin.rpy.rpyToMatrix(forearm_rpy),forearm_trans)

    wrist_1_trans = np.array([-0.3922353894477613, -0.001171506236920081, 0.1337997346972175])
    wrist_1_rpy = np.array([0.008755445624588536, 0.0002860523431017214, 7.215921353974553e-06])
    wrist_1_se3 = pin.SE3(pin.rpy.rpyToMatrix(wrist_1_rpy),wrist_1_trans)

    wrist_2_trans = np.array([5.620166987673597e-05, -0.09948910981796041, 0.0002201494606859632])
    wrist_2_rpy = np.array([1.568583530823855, 0, -3.513049549874747e-07])
    wrist_2_se3 = pin.SE3(pin.rpy.rpyToMatrix(wrist_2_rpy),wrist_2_trans)

    wrist_3_trans = np.array([9.062061300900664e-06, 0.09947787349620175, 0.0001411778743239612])
    wrist_3_rpy = np.array([1.572215514545703, 3.141592653589793, 3.141592633687631])
    wrist_3_se3 = pin.SE3(pin.rpy.rpyToMatrix(wrist_3_rpy),wrist_3_trans)

    model = None
    collision_model = None
    visual_model = None
    # this command just calls the ones below it. both are kept here
    # in case pinocchio people decide to change their api.
    #model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_path_absolute, mesh_dir_absolute)
    model = pin.buildModelFromUrdf(urdf_path_absolute)
    visual_model = pin.buildGeomFromUrdf(model, urdf_path_absolute, pin.GeometryType.VISUAL, None, mesh_dir_absolute)
    collision_model = pin.buildGeomFromUrdf(model, urdf_path_absolute, pin.GeometryType.COLLISION, None, mesh_dir_absolute)

    # updating joint placements.
    model.jointPlacements[1] = shoulder_se3
    model.jointPlacements[2] = upper_arm_se3
    model.jointPlacements[3] = forearm_se3
    model.jointPlacements[4] = wrist_1_se3
    model.jointPlacements[5] = wrist_2_se3
    model.jointPlacements[6] = wrist_3_se3 
    data = pin.Data(model)

    return model, collision_model, visual_model, data

