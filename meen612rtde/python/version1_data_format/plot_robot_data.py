import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('QtAgg')
import numpy as np

filename = 'robot_data.csv'
# filename = "good_data/joint4_first_chirp_with_pushes.csv"

(timestamp, actual_q_0, actual_q_1, actual_q_2, actual_q_3, actual_q_4, actual_q_5,
	actual_qd_0, actual_qd_1, actual_qd_2, actual_qd_3, actual_qd_4, actual_qd_5,
	actual_current_0, actual_current_1, actual_current_2, actual_current_3, actual_current_4, actual_current_5,
	actual_joint_voltage_0, actual_joint_voltage_1, actual_joint_voltage_2, actual_joint_voltage_3, actual_joint_voltage_4, actual_joint_voltage_5,
	target_q_0, target_q_1, target_q_2, target_q_3, target_q_4, target_q_5
	)=np.loadtxt(filename, delimiter=',', skiprows=1).T
# target_q = np.vstack([target_q_0, target_q_1, target_q_2, target_q_3, target_q_4, target_q_5]).T
# target_qd = np.vstack([target_qd_0, target_qd_1, target_qd_2, target_qd_3, target_qd_4, target_qd_5]).T
# target_qdd = np.vstack([target_qdd_0, target_qdd_1, target_qdd_2, target_qdd_3, target_qdd_4, target_qdd_5]).T
actual_q = np.vstack([actual_q_0, actual_q_1, actual_q_2, actual_q_3, actual_q_4, actual_q_5]).T
actual_qd = np.vstack([actual_qd_0, actual_qd_1, actual_qd_2, actual_qd_3, actual_qd_4, actual_qd_5]).T
actual_current = np.vstack([actual_current_0, actual_current_1, actual_current_2, actual_current_3, actual_current_4, actual_current_5]).T
# joint_control_output = np.vstack([joint_control_output_0, joint_control_output_1, joint_control_output_2, joint_control_output_3, joint_control_output_4, joint_control_output_5]).T
# actual_TCP_pose = np.vstack([actual_TCP_pose_0, actual_TCP_pose_1, actual_TCP_pose_2, actual_TCP_pose_3, actual_TCP_pose_4, actual_TCP_pose_5]).T
# actual_TCP_speed = np.vstack([actual_TCP_speed_0, actual_TCP_speed_1, actual_TCP_speed_2, actual_TCP_speed_3, actual_TCP_speed_4, actual_TCP_speed_5]).T
# actual_TCP_force = np.vstack([actual_TCP_force_0, actual_TCP_force_1, actual_TCP_force_2, actual_TCP_force_3, actual_TCP_force_4, actual_TCP_force_5]).T
# target_TCP_pose = np.vstack([target_TCP_pose_0, target_TCP_pose_1, target_TCP_pose_2, target_TCP_pose_3, target_TCP_pose_4, target_TCP_pose_5]).T
# target_TCP_speed = np.vstack([target_TCP_speed_0, target_TCP_speed_1, target_TCP_speed_2, target_TCP_speed_3, target_TCP_speed_4, target_TCP_speed_5]).T
# joint_temperatures = np.vstack([joint_temperatures_0, joint_temperatures_1, joint_temperatures_2, joint_temperatures_3, joint_temperatures_4, joint_temperatures_5]).T
# joint_mode = np.vstack([joint_mode_0, joint_mode_1, joint_mode_2, joint_mode_3, joint_mode_4, joint_mode_5]).T
actual_joint_voltage = np.vstack([actual_joint_voltage_0, actual_joint_voltage_1, actual_joint_voltage_2, actual_joint_voltage_3, actual_joint_voltage_4, actual_joint_voltage_5]).T
target_q = np.vstack([target_q_0, target_q_1, target_q_2, target_q_3, target_q_4, target_q_5]).T

print(f"{actual_q[0,:]=}")
print(f"{actual_q[-1,:]=}")

# plt.plot(timestamp[:-1]-timestamp[0], timestamp[1:]-timestamp[:-1])

# plt.figure()
# plt.plot(timestamp-timestamp[0], actual_current[:,0])


# plt.figure()

joi = 4 # joint of interest
## Parameter identification for joint current/torque model
e = target_q[:,joi]-actual_q[:,joi]
# thresh = 1e-6 # RMSE=0.057147932
# thresh = 1e-3 # RMSE=0.0567498
thresh = 1e-5 # RMSE=0.0540851
thresh = 1e-4 # RMSE=0.0456489
thresh = 5e-5 # RMSE=0.04713066
thresh = 8e-5 # RMSE=0.04536671
thresh = 9e-5 # RMSE=0.04510296
int_e = e
for i in range(1, len(e)):
	int_e[i]=int_e[i-1]+(timestamp[i]-timestamp[i-1])*e[i]
	if (int_e[i]>thresh): int_e[i]=thresh
	if (int_e[i]<-thresh): int_e[i]=-thresh


b = np.array(actual_current[:,joi]).reshape((-1,1))
A = np.hstack([np.ones(b.shape), 
	actual_qd[:,[joi]], 
	actual_qd[:,[joi]]/(np.abs(actual_qd[:,[joi]])+1e-4), 
	# (e/(np.abs(e)+1e-1)).reshape((-1,1)), 
	(e/(np.abs(e)+1e-3)).reshape((-1,1)),
	(target_q[:,[joi]]-actual_q[:,[joi]]), 
	# (target_q[:,[joi]]-actual_q[:,[joi]])/(np.abs((target_q[:,[joi]]-actual_q[:,[joi]]))+1e-4),
	# int_e.reshape((-1,1))
	])
# current ~= 0.004 + 4 qd + 0.06 sgn(qd) + 1.33 sgn(e) - 73 u
x = np.linalg.solve(A.T@A, A.T@b)

# RMSE=0.056749

resid = (b-A@x)
RMSE = float(np.sqrt(resid.T@resid/(len(b)))[0,0])
print(f"{x=}")
print(f"{RMSE=} amps")
# print(timestamp)
# plt.plot(timestamp-timestamp[0], target_q[:,0])/
# plt.title("Joint %d"%joi)
fig, axs = plt.subplots(3,1,sharex=True, num = "Joint %d"%joi)
axs[0].plot(timestamp-timestamp[0], actual_q[:,joi]-actual_q[0,joi], label="actual_q")
axs[0].plot(timestamp-timestamp[0], target_q[:,joi]-target_q[0,joi], label="target_q")
axs[1].plot(timestamp-timestamp[0], actual_qd[:,joi]-actual_qd[0,joi], label="actual_qd")
# axs[0].plot(timestamp-timestamp[0], actual_joint_voltage[:,joi]-actual_joint_voltage[0,joi], label="actual_joint_voltage")
axs[2].plot(timestamp-timestamp[0], actual_current[:,joi]-actual_current[0,joi], label="actual_current")
axs[2].plot(timestamp-timestamp[0], A[:,1:]@x[1:,:], label="model_current")
axs[0].legend()
axs[1].legend()
axs[2].legend()
axs[2].set_xlabel("time (s)")
axs[2].set_ylabel("current")
axs[1].set_ylabel("qd")
axs[0].set_ylabel("q")
# plt.plot(timestamp-timestamp[0], joint_temperatures[:,0])
plt.show()
