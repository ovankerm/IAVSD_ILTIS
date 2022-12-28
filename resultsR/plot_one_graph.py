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

test_PID = np.loadtxt('Test_PID_q.res')
acc_test_PID = np.loadtxt('Test_PID_qdd.res')

plt.figure("In-phase at 10m/s - Vert. Displ.")
plt.plot(test_PID[:, Id_x], test_PID[:, Id_y], 'k-', label='y_position')
# plt.plot(test_PID[:, Id_x], acc_test_PID[:, Id_y]/9.81, 'k-.', label='acceleration y_position [g]')
plt.hlines(0, 0.0, test_PID[-1, Id_x], linestyles=u'dashed')
# plt.xlim((0.0, 30.0))
# plt.ylim((-0.5, 0.5))
plt.xlabel('Distance [m]')
plt.ylabel('Vertical displacement of CG [mm]')
plt.legend()

plt.show()