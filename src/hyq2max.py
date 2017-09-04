#!/usr/bin/python
import dwl
import numpy as np
import math


class HyQ2Max():
    def __init__(self, urdf, yarf):
        #  Four-bar linkage mechanical constants
        self.akh = 0.522
        self.fkb = 0.694
        self.cj = 0.005
        self.jk = 0.360 + 0.041
        # The linkage bars
        self.ak = 0.04614
        self.bk = 0.03406
        self.ar = 0.056
        self.br = 0.056
        self.ak2 = self.ak ** 2
        self.bk2 = self.bk ** 2
        self.ar2 = self.ar ** 2
        self.br2 = self.br ** 2
        self.cyl_retracted = 0.3251

        # The triangle CJK; it is invariant regardless the linkage status
        self.ck2 = self.cj ** 2 + self.jk ** 2
        self.ck = math.sqrt(self.ck2)
        self.ckh = math.acos(self.jk / self.ck)

        self.LF_HFE_JOINT_MOTOR_OFF = -2.356
        self.LF_HAA_JOINT_MOTOR_OFF = -0.698

        self.fbs = dwl.FloatingBaseSystem()
        self.fbs.resetFromURDFFile(urdf, yarf)
        self.LF_HAA = self.fbs.getJointId('lf_haa_joint')
        self.LF_HFE = self.fbs.getJointId('lf_hfe_joint')
        self.LF_KFE = self.fbs.getJointId('lf_kfe_joint')
        self.LH_HAA = self.fbs.getJointId('lh_haa_joint')
        self.LH_HFE = self.fbs.getJointId('lh_hfe_joint')
        self.LH_KFE = self.fbs.getJointId('lh_kfe_joint')
        self.RF_HAA = self.fbs.getJointId('rf_haa_joint')
        self.RF_HFE = self.fbs.getJointId('rf_hfe_joint')
        self.RF_KFE = self.fbs.getJointId('rf_kfe_joint')
        self.RH_HAA = self.fbs.getJointId('rh_haa_joint')
        self.RH_HFE = self.fbs.getJointId('rh_hfe_joint')
        self.RH_KFE = self.fbs.getJointId('rh_kfe_joint')

        self.FBLStatus = {'theta': 0.0, 'cylinder': 0.0, 'lever': 0.0
                          }
        # self.theta = 0.0
        # self.cylinder = 0.0
        # self.lever = 0.0

    def knee_angle2fbl_angle(self, angle):
        return math.pi - abs(angle)

    def update_geometry(self, theta):

        bkh = theta + self.fkb

        # The triangle ABK
        akb = self.akh + bkh
        ab2 = self.ak2 + self.bk2 - 2 * self.ak * self.bk * math.cos(akb)
        kba = math.acos((self.bk2 + ab2 - self.ak2) / (2 * self.bk * math.sqrt(ab2)))

        # The triangle ABR
        arb = math.acos((self.ar2 + self.br2 - ab2) / (2 * self.ar * self.br))
        abr = (math.pi - arb) / 2

        # The triangle BKR
        if akb > math.pi:
            kbr = abr - kba
        else:
            kbr = abr + kba

        kr2 = self.bk2 + self.br2 - 2 * self.bk * self.br * math.cos(kbr)
        kr = math.sqrt(kr2)
        bkr = math.acos((self.bk2 + kr2 - self.br2) / (2 * self.bk * kr))

        # The triangle CKR
        ckr = bkh - bkr - self.ckh
        cr2 = self.ck ** 2 + kr2 - 2 * self.ck * kr * math.cos(ckr)

        # The triangle KPR
        self.FBLStatus['cylinder'] = math.sqrt(cr2)
        crk = math.acos((cr2 + kr2 - self.ck2) / (2 * self.FBLStatus['cylinder'] * kr))

        krp = math.pi - crk
        # pkr = math.pi / 2 - krp

        q = - (math.pi - theta)
        q2 = q * q
        q3 = q2 * q
        q4 = q3 * q
        q5 = q4 * q
        self.FBLStatus['lever'] = -0.0005 * q5 - 0.0043 * q4 - 0.0086 * q3 - 0.0032 * q2 - 0.0109 * q + 0.0063

    def getTorqueLimits(self, q):
        max_actuator_effort = np.array([120.0, 3600.0, 3600.0,
                                        120.0, 3600.0, 3600.0,
                                        120.0, 3600.0, 3600.0,
                                        120.0, 3600.0, 3600.0])
        torque_lim = np.zeros(12)
        for i in range(12):
            self.update_geometry(q[i])
            if self.FBLStatus['lever'] == 0:
                torque_lim[i] = max_actuator_effort[i]
            else:
                torque_lim[i] = max_actuator_effort[i] * self.FBLStatus['lever']

        return torque_lim

    def getTorqueLimit(self, joint_id, theta):
        max_actuator_effort = np.array([120.0, 3600.0, 3600.0,
                                        120.0, 3600.0, 3600.0,
                                        120.0, 3600.0, 3600.0,
                                        120.0, 3600.0, 3600.0])

        self.update_geometry(self.knee_angle2fbl_angle(theta))
        if self.FBLStatus['lever'] == 0:
            torque_lim = max_actuator_effort[joint_id]
        else:
            torque_lim = max_actuator_effort[joint_id] * self.FBLStatus['lever']

        return torque_lim
