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
# Plotting at 5 m/s in-phase cosine bump normal contact
# ==============================================================================

Bumpy_10m_s_Normal_contact_q = np.loadtxt('Bumpy_10m_s_Normal_contact_q.res')
Bumpy_10m_s_Simple_contact_q = np.loadtxt('Bumpy_10m_s_Simple_contact_q.res')

plt.figure("deplacement horizontal contacts")
plt.plot(Bumpy_10m_s_Normal_contact_q[:, Id_x], Bumpy_10m_s_Normal_contact_q[:, Id_y], 'b-', label='Premier modèle')
plt.plot(Bumpy_10m_s_Simple_contact_q[:, Id_x], Bumpy_10m_s_Simple_contact_q[:, Id_y], 'r-', label='Second modèle')
# plt.plot(x, y, 'k-.', label='vertical disturbance')
plt.hlines(0, 0.0, 30.0, linestyles=u'dashed')
plt.xlim((0.0, 30.0))
# plt.ylim((-50., 200.0))
plt.xlabel('Distance [m]')
plt.ylabel('Déplacement horizontal [m]')
plt.legend()

# plt.figure("In-phase at 5m/s - Normal contact - Pitch Angle")
# plt.plot(InPhase5ms_normal_contact_q[:, Id_x], (InPhase5ms_normal_contact_q[:, Id_pitch] / np.pi) * 180., 'b-', label='Normal contact')
# plt.plot(InPhase5ms_simple_contact_q[:, Id_x], (InPhase5ms_simple_contact_q[:, Id_pitch] / np.pi) * 180., 'k-', label='Simple contact')
# plt.hlines(0, 0.0, 10.0, linestyles=u'dashed')
# plt.xlim((0.0, 10.0))
# plt.ylim((-10., 10.0))
# plt.xlabel('Distance [m]')
# plt.ylabel('Pitch angle of vehicle [deg]')
# plt.legend()

# plt.figure("In-phase at 5m/s - Normal contact - Vertical tyre forces")
# plt.plot(InPhase5ms_normal_contact_q[:, Id_x], -InPhase5ms_normal_contact_wheels[:, Id_RR], 'b--', label='Right rear  wheel - Normal contact')
# plt.plot(InPhase5ms_normal_contact_q[:, Id_x], -InPhase5ms_normal_contact_wheels[:, Id_FR], 'b-', label='Right front wheel - Normal contact')
# plt.plot(InPhase5ms_simple_contact_q[:, Id_x], -InPhase5ms_simple_contact_wheels[:, Id_RR], 'k--', label='Right rear  wheel - Simple contact')
# plt.plot(InPhase5ms_simple_contact_q[:, Id_x], -InPhase5ms_simple_contact_wheels[:, Id_FR], 'k-', label='Right front wheel - Simple contact')
# plt.xlim((0.0, 10.0))
# plt.ylim((-1.0e4, 0.0))
# plt.xlabel('Distance [m]')
# plt.ylabel('Vertical tire force [N]')
# plt.legend()

plt.show()

# # ==============================================================================
# # Plotting at 5 m/s in-phase cosine bump simple contact
# # ==============================================================================

# InPhase5ms_q = np.loadtxt('InPhaseCosine_5ms_simple_contact_q.res')
# InPhase5ms_wheels = np.loadtxt('InPhaseCosine_5ms_simple_contact_Ground_Forces.res')

# plt.figure("In-phase at 5m/s - Simple Contact - Vert. Displ.")
# plt.plot(InPhase5ms_q[:, Id_x], (InPhase5ms_q[:, Id_z] - 0.57) * 1000., 'k-', label='Robotran : speed 5m/s')
# plt.plot(x, y, 'k-.', label='vertical disturbance')
# plt.hlines(0, 0.0, 10.0, linestyles=u'dashed')
# plt.xlim((0.0, 10.0))
# plt.ylim((-50., 200.0))
# plt.xlabel('Distance [m]')
# plt.ylabel('Vertical displacement of CG [mm]')
# plt.legend()

# plt.figure("In-phase at 5m/s - Simple Contact - Pitch Angle")
# plt.plot(InPhase5ms_q[:, Id_x], (InPhase5ms_q[:, Id_pitch] / np.pi) * 180., 'k-', label='Robotran : speed 5m/s')
# plt.hlines(0, 0.0, 10.0, linestyles=u'dashed')
# plt.xlim((0.0, 10.0))
# plt.ylim((-10., 10.0))
# plt.xlabel('Distance [m]')
# plt.ylabel('Pitch angle of vehicle [deg]')
# plt.legend()

# plt.figure("In-phase at 5m/s - Simple Contact - Vertical tyre forces")
# plt.plot(InPhase5ms_q[:, Id_x], -InPhase5ms_wheels[:, Id_RR], 'k--', label='Robotran : Right rear  wheel')
# plt.plot(InPhase5ms_q[:, Id_x], -InPhase5ms_wheels[:, Id_FR], 'k-', label='Robotran : Right front wheel')
# plt.xlim((0.0, 10.0))
# plt.ylim((-1.0e4, 0.0))
# plt.xlabel('Distance [m]')
# plt.ylabel('Vertical tire force [N]')
# plt.legend()

# plt.show()