import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from drake_controller_wrapper import FeedbackControllerDrakeSimWrapper

print(plt.get_backend())
print(mpl.rcsetup.interactive_bk)
mpl.use("qtagg")

state_data = np.loadtxt("state_data.csv", delimiter=",")
contr_data = np.loadtxt("contr_data.csv", delimiter=",")
state_times = np.loadtxt("state_times.csv", delimiter=",")
contr_times = np.loadtxt("contr_times.csv", delimiter=",")
grav_comp_controller = FeedbackControllerDrakeSimWrapper()

fig, ax = plt.subplots(3,1,sharex=True)
#state_names = plant.GetStateNames() # print this out if you're curious, or use pythons '.index()' function to plot specifics
#actuator_names = [plant.get_joint_actuator(i).name() for i in plant.GetJointActuatorIndices()]

print(f"{state_data[0,:6]}")

qs = state_data[:,:6]
qds = state_data[:,6:]
taus = contr_data[:,:]

tau_grav = np.array([grav_comp_controller.GetInitialGravity(q, qd) for (q, qd) in zip(qs, qds)])

ax[0].plot(state_times, state_data[:,:6])#, label=state_names)
ax[1].plot(state_times, state_data[:,6:])#, label=state_names)
ax[2].plot(contr_times, taus-tau_grav)#, label=actuator_names)
#ax[0].legend()
ax[0].grid()
ax[0].set_xlabel('time (s)')
ax[0].set_ylabel('positions')
#ax[1].legend()
ax[1].grid()
ax[1].set_xlabel('time (s)')
ax[1].set_ylabel('velocities')
ax[2].grid()
ax[2].set_xlabel('time (s)')
ax[2].set_ylabel('torques minus gravity comp')

plt.figure(num="Time Steps")
plt.plot(contr_times[1:], contr_times[1:]-contr_times[:-1])

plt.show()
