#!/usr/bin/python
# dwl modules
import dwl
import numpy as np
import roslib
roslib.load_manifest('dwl_msgs')
from dwl_msgs import BagParser as bag_parser

# dls-python-utils modules
import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))) # import the classes inside the src folder
from src import hyq2max, array_utils

# matplotlib modules
import matplotlib.patches as mpatches
import matplotlib as mpl
import matplotlib.pyplot as plt



dwl_path = '/home/aradulescu/catkin_ws/src/dls/dwl-distro/dwl/'
initial_time = 14.314 + 15
duration = 5 #25.
bag_file = '/home/aradulescu/ownCloud/OneShotDocumentation/PoseRec_VariableBlocks/oneshot_opt_poseRecovery_2017-Jul-31-16-45.bag'
topic = '/hyq/robot_states'
leg_name = 'rf'


# Creating the hyq class and its floating-base model
urdf = dwl_path + 'sample/hyq2max.urdf'
yarf = dwl_path + 'config/hyq2max.yarf'

robot = hyq2max.HyQ2Max(urdf, yarf)
fbs = dwl.FloatingBaseSystem()
fbs.resetFromURDFFile(dwl_path + 'sample/hyq2max.urdf',
                      dwl_path + 'config/hyq2max.yarf')