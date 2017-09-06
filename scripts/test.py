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
plt.close()

dwl_path = '/home/aradulescu/catkin_ws/src/dls/dwl-distro/dwl/'
initial_time = 2.05
duration = 3 #25.
bag_file = '/home/aradulescu/ownCloud/OneShotDocumentation/PoseRec_VariableBlocks/oneshot_opt_poseRecovery_2017-Jul-31-16-45.bag'
topic = '/hyq2max/robot_states'
leg_name = 'rf'


# Creating the hyq class and its floating-base model
urdf = dwl_path + 'sample/hyq2max.urdf'
yarf = dwl_path + 'config/hyq2max.yarf'

robot = hyq2max.HyQ2Max(urdf, yarf)
fbs = dwl.FloatingBaseSystem()
fbs.resetFromURDFFile(dwl_path + 'sample/hyq2max.urdf',
                      dwl_path + 'config/hyq2max.yarf')


# Extrating the whole-body trajectory
ws_vec = bag_parser.extractWholeBodyState(bag_file,
                                          topic,
                                          initial_time,
                                          duration)

time = array_utils.getTimeArray(ws_vec)
joint_pos = array_utils.getJointPositionArray(ws_vec)
num_joints = ws_vec[0].getJointDoF()

#  = [fbs.getJointLimits(i) for i in range(num_joints)]
# for i in range(num_joints):
joint_limits= fbs.getJointLimit('rf_hfe_joint')
print "upper %f" % joint_limits.lower
# Plotting the joint positions
fig, ax = plt.subplots(nrows=1, sharex=True)
ax.plot(time, joint_pos[fbs.getJointId('rf_hfe_joint')], '#00008B', linewidth=1., label=r"rf_hfe_joint")
ax.plot(time, joint_pos[fbs.getJointId('rh_hfe_joint')], '#D2691E', linewidth=1., label=r"rh_hfe_joint")
ax.plot(time, joint_pos[fbs.getJointId('lf_hfe_joint')], '#8B0000', linewidth=1., label=r"lf_hfe_joint")
ax.plot(time, joint_pos[fbs.getJointId('lh_hfe_joint')], '#008000', linewidth=1., label=r"lh_hfe_joint")
ax.set_title('Joint positions', fontsize=18)
ax.set_xlabel('Time (ms)', {'color':'k', 'fontsize':12})
ax.set_ylabel(r'KFE (Nm)', {'color':'k', 'fontsize':12})


# handles, labels = ax.get_legend_handles_labels()
# reverse the order
# ax.legend(handles[::-1], labels[::-1])
# or sort them by labels
# import operator
# hl = sorted(zip(handles, labels),
#             key=operator.itemgetter(1))
# handles2, labels2 = zip(*hl)
#
# ax.legend(handles2, labels2)
ax.legend(loc='center')
ax.grid(True)
# plt.axis('equal')
# ax.set_ylim([1.0, 1.75])
ax.spines['right'].set_visible(False)
ax.spines['top'].set_visible(False)
plt.show()



print "The LF_HAA angle is %d" % robot.LF_HAA
