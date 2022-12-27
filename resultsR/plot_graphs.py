# -*- coding: utf-8 -*-
"""Plot the simulation results according to the benchmark instructions.

@author: olantsoght
"""

import numpy as np
import matplotlib.pyplot as plt
import os

os.chdir('/Users/vankermotis/Documents/MBProjects/IAVSD_ILTIS/resultsR/')

Id_x = 1
Id_y = 2
Id_z = 3
Id_pitch = 4
Id_roll = 5
Id_yaw = 6
Id_FR = 2
Id_RR = 3
Id_FL = 1
Id_RL = 4

x = np.linspace(0.97, 5.97, 2000)
y = 0.1 * (1 - np.cos(2 * np.pi * (x - 0.97) / 5)) * 1000


# ==============================================================================
# Plotting at 5 m/s in-phase cosine bump
# ==============================================================================

InPhase5ms_q = np.loadtxt('InPhaseCosine_5ms_q.res')
InPhase5ms_wheels = np.loadtxt('InPhaseCosine_5ms_Ground_Forces.res')

plt.figure("In-phase at 5m/s - Vert. Displ.")
plt.plot(InPhase5ms_q[:, Id_x], (InPhase5ms_q[:, Id_z] - 0.57) * 1000., 'k-', label='Robotran : speed 5m/s')
plt.plot(x, y, 'k-.', label='vertical disturbance')
plt.hlines(0, 0.0, 10.0, linestyles=u'dashed')
plt.xlim((0.0, 10.0))
plt.ylim((-50., 200.0))
plt.xlabel('Distance [m]')
plt.ylabel('Vertical displacement of CG [mm]')
plt.legend()
plt.savefig('In-phase_at_5ms_Vert_Displ.png')

plt.figure("In-phase at 5m/s - Pitch Angle")
plt.plot(InPhase5ms_q[:, Id_x], (InPhase5ms_q[:, Id_pitch] / np.pi) * 180., 'k-', label='Robotran : speed 5m/s')
plt.hlines(0, 0.0, 10.0, linestyles=u'dashed')
plt.xlim((0.0, 10.0))
plt.ylim((-10., 10.0))
plt.xlabel('Distance [m]')
plt.ylabel('Pitch angle of vehicle [deg]')
plt.legend()
plt.savefig('In-phase_at_5ms_Pitch_Angle.png')

plt.figure("In-phase at 5m/s - Vertical tyre forces")
plt.plot(InPhase5ms_q[:, Id_x], -InPhase5ms_wheels[:, Id_RR], 'k--', label='Robotran : Right rear  wheel')
plt.plot(InPhase5ms_q[:, Id_x], -InPhase5ms_wheels[:, Id_FR], 'k-', label='Robotran : Right front wheel')
plt.xlim((0.0, 10.0))
plt.ylim((-1.0e4, 0.0))
plt.xlabel('Distance [m]')
plt.ylabel('Vertical tire force [N]')
plt.legend()
plt.savefig('In-phase_at_5ms_Tyre_Forces.png')


# ==============================================================================
# Plotting at 10 m/s in-phase cosine bump
# ==============================================================================

InPhase10ms_q = np.loadtxt('InPhaseCosine_10ms_q.res')
InPhase10ms_wheels = np.loadtxt('InPhaseCosine_10ms_Ground_Forces.res')
Leaf = np.loadtxt('InPhaseCosine_10ms_Leaf_Forces.res')

plt.figure("In-phase at 10m/s - Vert. Displ.")
plt.plot(InPhase10ms_q[:, Id_x], (InPhase10ms_q[:, Id_z] - 0.57) * 1000., 'k-', label='Robotran : speed 10m/s')
plt.plot(x, y, 'k-.', label='vertical disturbance')
plt.hlines(0, 0.0, 20.0, linestyles=u'dashed')
plt.xlim((0.0, 20.0))
plt.ylim((-100., 350.0))
plt.xlabel('Distance [m]')
plt.ylabel('Vertical displacement of CG [mm]')
plt.legend()
plt.savefig('In-phase_at_10ms_Vert_Displ.png')

plt.figure("In-phase at 10m/s - Pitch Angle")
plt.plot(InPhase10ms_q[:, Id_x], (InPhase10ms_q[:, Id_pitch] / np.pi) * 180., 'k-', label='Robotran : speed 10m/s')
plt.hlines(0, 0.0, 20.0, linestyles=u'dashed')
plt.xlim((0.0, 20.0))
plt.ylim((-10., 10.0))
plt.xlabel('Distance [m]')
plt.ylabel('Pitch angle of vehicle [deg]')
plt.legend()
plt.savefig('In-phase_at_10ms_Pitch_Angle.png')

plt.figure("In-phase at 10m/s - Vertical tyre forces")
plt.plot(InPhase10ms_q[:, Id_x], -InPhase10ms_wheels[:, Id_RR], 'k--', label='Robotran : Right rear  wheel')
plt.plot(InPhase10ms_q[:, Id_x], -InPhase10ms_wheels[:, Id_FR], 'k-', label='Robotran : Right front wheel')
plt.hlines(0, 0.0, 20.0, linestyles=u'dashed')
plt.xlim((0.0, 20.0))
plt.ylim((-3.5e4, 5000.0))
plt.xlabel('Distance [m]')
plt.ylabel('Vertical tire force [N]')
plt.legend()
plt.savefig('In-phase_at_10ms_Tyre_Forces.png')


# ==============================================================================
# Plotting at 5 m/s anti-phase cosine bump
# ==============================================================================

OutPhase5ms_q = np.loadtxt('AntiPhaseCosine_5ms_q.res')
OutPhase5ms_wheels = np.loadtxt('AntiPhaseCosine_5ms_Ground_Forces.res')

plt.figure("Anti-phase at 5m/s - Roll Angle")
plt.plot(OutPhase5ms_q[:, Id_x], -(OutPhase5ms_q[:, Id_roll] / np.pi) * 180., 'k-', label='Robotran : speed 5m/s')
plt.hlines(0, 0.0, 13.0, linestyles=u'dashed')
plt.xlim((0.0, 13.0))
plt.ylim((-25., 5.0))
plt.xlabel('Distance [m]')
plt.ylabel('Roll angle of vehicle [deg]')
plt.legend()
plt.savefig('Anti-phase_at_5ms_Roll_Angle.png')

plt.figure("Anti-phase at 5m/s - Yaw Angle")
plt.plot(OutPhase5ms_q[:, Id_x], (OutPhase5ms_q[:, Id_yaw] / np.pi) * 180., 'k-', label='Robotran : speed 5m/s')
plt.hlines(0, 0.0, 13.0, linestyles=u'dashed')
plt.xlim((0.0, 13.0))
plt.ylim((-4., 4.0))
plt.xlabel('Distance [m]')
plt.ylabel('Yaw angle of vehicle [deg]')
plt.legend()
plt.savefig('Anti-phase_at_5ms_Yaw_Angle.png')

plt.figure("Anti-phase at 5m/s - Vertical tyre forces")
plt.plot(OutPhase5ms_q[:, Id_x], -OutPhase5ms_wheels[:, Id_RR], 'k--', label='Robotran : Right rear  wheel')
plt.plot(OutPhase5ms_q[:, Id_x], -OutPhase5ms_wheels[:, Id_FR], 'k-', label='Robotran : Right front wheel')
plt.plot(OutPhase5ms_q[:, Id_x], -OutPhase5ms_wheels[:, Id_RL], 'k-.', label='Robotran : Left rear  wheel')
plt.plot(OutPhase5ms_q[:, Id_x], -OutPhase5ms_wheels[:, Id_FL], 'k:', label='Robotran : Left front wheel')
plt.hlines(0, 0.0, 13.0, linestyles=u'dashed')
plt.xlim((0.0, 13.0))
plt.ylim((-3.0e4, 5000.0))
plt.xlabel('Distance [m]')
plt.ylabel('Vertical tire force [N]')
plt.legend()
plt.savefig('Anti-phase_at_5ms_Tyre_Forces.png')


# ==============================================================================
# Plotting at 10 m/s anti-phase cosine bump
# ==============================================================================

OutPhase10ms_q = np.loadtxt('AntiPhaseCosine_10ms_q.res')
OutPhase10ms_wheels = np.loadtxt('AntiPhaseCosine_10ms_Ground_Forces.res')

plt.figure("Anti-phase at 10m/s - Roll Angle")
plt.plot(OutPhase10ms_q[:, Id_x], -(OutPhase10ms_q[:, Id_roll] / np.pi) * 180., 'k-', label='Robotran : speed 10m/s')
plt.hlines(0, 0.0, 13.0, linestyles=u'dashed')
plt.xlim((0.0, 13.0))
plt.ylim((-25., 5.0))
plt.xlabel('Distance [m]')
plt.ylabel('Roll angle of vehicle [deg]')
plt.legend()
plt.savefig('Anti-phase_at_10ms_Roll_Angle.png')

plt.figure("Anti-phase at 10m/s - Yaw Angle")
plt.plot(OutPhase10ms_q[:, Id_x], (OutPhase10ms_q[:, Id_yaw] / np.pi) * 180., 'k-', label='Robotran : speed 10m/s')
plt.hlines(0, 0.0, 13.0, linestyles=u'dashed')
plt.xlim((0.0, 13.0))
plt.ylim((-4., 4.0))
plt.xlabel('Distance [m]')
plt.ylabel('Yaw angle of vehicle [deg]')
plt.legend()
plt.savefig('Anti-phase_at_10ms_Yaw_Angle.png')

plt.figure("Anti-phase at 10m/s - Vertical tyre forces")
plt.plot(OutPhase10ms_q[:, Id_x], -OutPhase10ms_wheels[:, Id_RR], 'k--', label='Robotran : Right rear  wheel')
plt.plot(OutPhase10ms_q[:, Id_x], -OutPhase10ms_wheels[:, Id_FR], 'k-', label='Robotran : Right front wheel')
plt.plot(OutPhase10ms_q[:, Id_x], -OutPhase10ms_wheels[:, Id_RL], 'k-.', label='Robotran : Left rear  wheel')
plt.plot(OutPhase10ms_q[:, Id_x], -OutPhase10ms_wheels[:, Id_FL], 'k:', label='Robotran : Left front wheel')
plt.hlines(0, 0.0, 13.0, linestyles=u'dashed')
plt.xlim((0.0, 13.0))
plt.ylim((-3.0e4, 5000.0))
plt.xlabel('Distance [m]')
plt.ylabel('Vertical tire force [N]')
plt.legend()
plt.savefig('Anti-phase_at_10ms_Tyre_Forces.png')


# ==============================================================================
# Plotting Ramp to Steer
# ==============================================================================

R2S10ms_q = np.loadtxt('RampToSteer_10ms_q.res')
R2S20ms_q = np.loadtxt('RampToSteer_20ms_q.res')
R2S30ms_q = np.loadtxt('RampToSteer_30ms_q.res')
R2S10ms_qd = np.loadtxt('RampToSteer_10ms_qd.res')
R2S20ms_qd = np.loadtxt('RampToSteer_20ms_qd.res')
R2S30ms_qd = np.loadtxt('RampToSteer_30ms_qd.res')
R2S10ms_qdd = np.loadtxt('RampToSteer_10ms_qdd.res')
R2S20ms_qdd = np.loadtxt('RampToSteer_20ms_qdd.res')
R2S30ms_qdd = np.loadtxt('RampToSteer_30ms_qdd.res')

plt.figure("Ramp to Steer 4 mm - Yaw Rate")
plt.plot(R2S10ms_qd[:, 0], R2S10ms_qd[:, Id_yaw], 'k-', label='Robotran : speed 10m/s')
plt.plot(R2S20ms_qd[:, 0], R2S20ms_qd[:, Id_yaw], 'k-.', label='Robotran : speed 20m/s')
plt.plot(R2S30ms_qd[:, 0], R2S30ms_qd[:, Id_yaw], 'k:', label='Robotran : speed 30m/s')
plt.hlines(0, 0.0, 5.0, linestyles=u'dashed')
plt.xlim((0.0, 5.0))
plt.ylim((-0.12, 0.02))
plt.xlabel('Simulation time [s]')
plt.ylabel('Yaw rate of vehicle [rad/s]')
plt.legend()
plt.savefig('RampToSteer_Yaw_Rate.png')

plt.figure("Ramp to Steer 4 mm - Lateral Acc.")
plt.plot(R2S10ms_qdd[:, 0], -np.linalg.norm(R2S10ms_qdd[:, [Id_x, Id_y]], axis=1), 'k-', label='Robotran : speed 10m/s')
plt.plot(R2S20ms_qdd[:, 0], -np.linalg.norm(R2S20ms_qdd[:, [Id_x, Id_y]], axis=1), 'k-', label='Robotran : speed 20m/s')
plt.plot(R2S30ms_qdd[:, 0], -np.linalg.norm(R2S30ms_qdd[:, [Id_x, Id_y]], axis=1), 'k-', label='Robotran : speed 30m/s')
plt.hlines(0, 0.0, 5.0, linestyles=u'dashed')
plt.xlim((0.0, 5.0))
plt.ylim((-3.0, 0.5))
plt.xlabel('Distance [m]')
plt.ylabel('Yaw angle of vehicle [deg]')
plt.legend()
plt.savefig('RampToSteer_Lateral_Acc.png')


# ==============================================================================
# Plotting Sinusoidal steering
# ==============================================================================

SS1mm_qd = np.loadtxt('SinusoidalSteer_1mm_qd.res')
SS2mm_qd = np.loadtxt('SinusoidalSteer_2mm_qd.res')
SS4mm_qd = np.loadtxt('SinusoidalSteer_4mm_qd.res')
SS1mm_qdd = np.loadtxt('SinusoidalSteer_1mm_qdd.res')
SS2mm_qdd = np.loadtxt('SinusoidalSteer_2mm_qdd.res')
SS4mm_qdd = np.loadtxt('SinusoidalSteer_4mm_qdd.res')

plt.figure("Sinusoidal steering at 20 m/s - Yaw Rate")
plt.plot(SS1mm_qd[:, 0], SS1mm_qd[:, Id_yaw], 'k-', label='Robotran : amplitude 1mm')
plt.plot(SS2mm_qd[:, 0], SS2mm_qd[:, Id_yaw], 'k-.', label='Robotran : amplitude 2mm')
plt.plot(SS4mm_qd[:, 0], SS4mm_qd[:, Id_yaw], 'k:', label='Robotran : amplitude 4mm')
plt.hlines(0, 0.0, 5.0, linestyles=u'dashed')
plt.xlim((0.0, 3.0))
plt.ylim((-0.2, 0.2))
plt.xlabel('Simulation time [s]')
plt.ylabel('Yaw rate of vehicle [rad/s]')
plt.legend()
plt.savefig('SinusoidalSteering_Yaw_Rate_20ms.png')

plt.figure("Sinusoidal steering at 20 m/s - Lateral Acc.")
plt.plot(SS1mm_qdd[:, 0], np.linalg.norm(SS1mm_qdd[:, [Id_x, Id_y]], axis=1), 'k-', label='Robotran : amplitude 1mm')
plt.plot(SS2mm_qdd[:, 0], np.linalg.norm(SS2mm_qdd[:, [Id_x, Id_y]], axis=1), 'k-.', label='Robotran : amplitude 2mm')
plt.plot(SS4mm_qdd[:, 0], np.linalg.norm(SS4mm_qdd[:, [Id_x, Id_y]], axis=1), 'k:', label='Robotran : amplitude 4mm')
plt.hlines(0, 0.0, 5.0, linestyles=u'dashed')
plt.xlim((0.0, 3.0))
plt.ylim((-2.5, 2.5))
plt.xlabel('Simulation time [s]')
plt.ylabel('Lateral acceleration (norm) of vehicle [m/(s*s)]')
plt.legend()
plt.savefig('SinusoidalSteering_Lateral_Acc_20ms.png')


plt.show()
