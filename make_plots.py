import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

print(plt.get_backend())
print(mpl.rcsetup.interactive_bk)
mpl.use("qtagg")

state_data = np.loadtxt("state_data.csv", delimiter=",")
contr_data = np.loadtxt("contr_data.csv", delimiter=",")
state_times = np.loadtxt("state_times.csv", delimiter=",")
contr_times = np.loadtxt("contr_times.csv", delimiter=",")


fig, ax = plt.subplots(3,1,sharex=True)
#state_names = plant.GetStateNames() # print this out if you're curious, or use pythons '.index()' function to plot specifics
#actuator_names = [plant.get_joint_actuator(i).name() for i in plant.GetJointActuatorIndices()]

ax[0].plot(state_times, state_data[:,:6])#, label=state_names)
ax[1].plot(state_times, state_data[:,6:])#, label=state_names)
ax[2].plot(contr_times, contr_data)#, label=actuator_names)
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
ax[2].set_ylabel('commanded torques')

plt.figure(num="Time Steps")
plt.plot(contr_times[1:], contr_times[1:]-contr_times[:-1])

plt.show()
