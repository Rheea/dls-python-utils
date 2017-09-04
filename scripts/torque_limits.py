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
from src import hyq, array_utils

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
urdf = dwl_path + 'sample/hyq.urdf'
yarf = dwl_path + 'config/hyq.yarf'
robot = hyq.HyQ(urdf, yarf)
fbs = dwl.FloatingBaseSystem()
fbs.resetFromURDFFile(dwl_path + 'sample/hyq.urdf',
                      dwl_path + 'config/hyq.yarf')


# Extrating the whole-body trajectory
ws_vec = bag_parser.extractWholeBodyState(bag_file,
                                          topic,
                                          initial_time,
                                          duration)


time = array_utils.getTimeArray(ws_vec)
#joint_eff = array_utils.getJointEffortArray(ws_vec)
#num_joints = ws_vec[0].getJointDoF()
#joint_lbound = [[-robot.getTorqueLimit(i, k.getJointPosition()[i]) for k in ws_vec] for i in range(num_joints)]
#joint_ubound = [[robot.getTorqueLimit(i, k.getJointPosition()[i]) for k in ws_vec] for i in range(num_joints)]


# Plotting the torques leg_name HFE
fig, ax = plt.subplots(nrows=1, sharex=True)
#ax.plot(time, joint_lbound[fbs.getJointId(leg_name + '_hfe_joint')], '#ff8000', linewidth=2., label=r"lims")
#ax.plot(time, joint_ubound[fbs.getJointId(leg_name + '_hfe_joint')], '#ff8000', linewidth=2.)
#ax.plot(time, joint_eff[fbs.getJointId(leg_name + '_hfe_joint')], '#ff0000', linewidth=2., label=r"cmd")
ax.axhspan(90., 150.,  color='b', alpha=0.15, lw=0)
ax.axhspan(-90., -150.,  color='b', alpha=0.15, lw=0)
ax.set_title('Joint torques', fontsize=18)
ax.set_ylabel(r'KFE (Nm)', {'color':'k', 'fontsize':12})
ax.grid(True)
ax.set_ylim([-150, 150])
ax.spines['right'].set_visible(False)
ax.spines['top'].set_visible(False)
plt.show()
