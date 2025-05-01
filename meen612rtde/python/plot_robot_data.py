import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('QtAgg')
import numpy as np

from meen612_ID import InverseDynamics

# from my_admittance import 

jois = range(6)#2 # joint of interest
filename = 'robot_data.csv'
# filename = "good_data/joint4_first_chirp_with_pushes.csv"
# filename = "good_data/j4_sysid2.csv"  
# filename = "good_data/j4_sys_ID_no_envelope.csv"
# filename = "good_data/whole_robot_1.csv"

# Literature values from
# Andrea Raviola *, Roberto Guida, Andrea De Martin, Stefano Pastorelli, Stefano Mauro and Massimo Sorli
# "Effects of Temperature and Mounting Configuration on the Dynamic Parameters Identification of Industrial Robots"
# Robotics 2021, 10, 83. https://doi.org/10.3390/robotics10030083 (MDPI)
# See Table 1 on page 4
literature_G = 101.0
literature_Ktau = np.array([0.1350, 0.1361, 0.1355, 0.0957, 0.0865, 0.0893])*literature_G

(timestamp, actual_q_0, actual_q_1, actual_q_2, actual_q_3, actual_q_4, actual_q_5,
    actual_qd_0, actual_qd_1, actual_qd_2, actual_qd_3, actual_qd_4, actual_qd_5,
    actual_current_0, actual_current_1, actual_current_2, actual_current_3, actual_current_4, actual_current_5,
    actual_joint_voltage_0, actual_joint_voltage_1, actual_joint_voltage_2, actual_joint_voltage_3, actual_joint_voltage_4, actual_joint_voltage_5,
    software_target_q_0, software_target_q_1, software_target_q_2, software_target_q_3, software_target_q_4, software_target_q_5,
    target_current_0, target_current_1, target_current_2, target_current_3, target_current_4, target_current_5,
    target_moment_0, target_moment_1, target_moment_2, target_moment_3, target_moment_4, target_moment_5,
    target_q_0, target_q_1, target_q_2, target_q_3, target_q_4, target_q_5,
    target_qd_0, target_qd_1, target_qd_2, target_qd_3, target_qd_4, target_qd_5,
    target_qdd_0, target_qdd_1, target_qdd_2, target_qdd_3, target_qdd_4, target_qdd_5,
    ctrl_u_0, ctrl_u_1, ctrl_u_2, ctrl_u_3, ctrl_u_4, ctrl_u_5,
    sim_q_0, sim_q_1, sim_q_2, sim_q_3, sim_q_4, sim_q_5
    )=np.loadtxt(filename, delimiter=',', skiprows=1).T
target_q = np.vstack([target_q_0, target_q_1, target_q_2, target_q_3, target_q_4, target_q_5]).T
target_qd = np.vstack([target_qd_0, target_qd_1, target_qd_2, target_qd_3, target_qd_4, target_qd_5]).T
target_qdd = np.vstack([target_qdd_0, target_qdd_1, target_qdd_2, target_qdd_3, target_qdd_4, target_qdd_5]).T
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
software_target_q = np.vstack([software_target_q_0, software_target_q_1, software_target_q_2, software_target_q_3, software_target_q_4, software_target_q_5]).T
target_current = np.vstack([target_current_0, target_current_1, target_current_2, target_current_3, target_current_4, target_current_5]).T
target_moment = np.vstack([target_moment_0, target_moment_1, target_moment_2, target_moment_3, target_moment_4, target_moment_5]).T
ctrl_u = np.vstack([ctrl_u_0, ctrl_u_1, ctrl_u_2, ctrl_u_3, ctrl_u_4, ctrl_u_5]).T
sim_q = np.vstack([sim_q_0, sim_q_1, sim_q_2, sim_q_3, sim_q_4, sim_q_5]).T
print(f"{actual_q[0,:]=}")
print(f"{actual_q[-1,:]=}")

id_rob = InverseDynamics()

# plt.plot(timestamp[:-1]-timestamp[0], timestamp[1:]-timestamp[:-1])

# plt.figure()
# plt.plot(timestamp-timestamp[0], actual_current[:,0])

id_torques = np.zeros(target_q.shape)
for i in range(target_q.shape[0]):
    q = target_q[i,:]
    qd = target_qd[i,:]
    qdd = target_qdd[i,:]
    id_torques[i,:] = id_rob.calc_ID_tau(q, qd, qdd)


# plt.figure()

for joi in jois:

    ## Parameter identification for the position prefilter
    b_00 = np.array(target_qdd[:,joi]).reshape((-1,1))
    A_00 = np.hstack([np.ones(b_00.shape), 
        np.roll(software_target_q[:,[joi]],2,axis=0), 
        np.roll(target_q[:,[joi]],1,axis=0), 
        np.roll(target_qd[:,[joi]],1,axis=0)
        ])
    x_00 = np.linalg.solve(A_00.T@A_00, A_00.T@b_00)
    resid_00 = (b_00-A_00@x_00)
    RMSE_00 = float(np.sqrt(resid_00.T@resid_00/(len(b_00)))[0,0])
    print(f"{x_00=}")
    print(f"{RMSE_00=} rad/s/s")
    # qdd_des = -0.0442 + 99.7 u(t-2) - 99.7 q_des(t-1) - 3.12 qd_des(t-1)
    # x_00=array([[-4.42689041e-02],
    #        [ 9.97128339e+01],
    #        [-9.96845232e+01],
    #        [-3.11808602e+00]])




    b_01 = np.array(target_qd[:,joi]).reshape((-1,1))
    A_01 = np.hstack([
        np.roll(target_qdd[:,[joi]],1,axis=0), 
        # np.roll(target_q[:,[joi]],1,axis=0), 
        np.roll(target_qd[:,[joi]],1,axis=0)
        ])
    x_01 = np.linalg.solve(A_01.T@A_01, A_01.T@b_01)
    resid_01 = (b_01-A_01@x_01)
    RMSE_01 = float(np.sqrt(resid_01.T@resid_01/(len(b_01)))[0,0])
    print(f"{x_01=}")
    print(f"{RMSE_01=} rad/s")
    # qd_des = 0.002 qdd_des(t-1) + qd_des(t-1) ## Euler integration
    # x_01=array([[0.00199323],
    #        [0.9995664 ]])

    b_02 = np.array(target_q[:,joi]).reshape((-1,1))
    A_02 = np.hstack([
        np.roll(target_qdd[:,[joi]],1,axis=0), 
        np.roll(target_qd[:,[joi]],1,axis=0),
        np.roll(target_q[:,[joi]],1,axis=0)
        ])
    x_02 = np.linalg.solve(A_02.T@A_02, A_02.T@b_02)
    resid_02 = (b_02-A_02@x_02)
    RMSE_02 = float(np.sqrt(resid_02.T@resid_02/(len(b_02)))[0,0])
    print(f"{x_02=}")
    print(f"{RMSE_02=} rad")
    # q_des = q_des(t-1) + 0.002 qd_des(t-1) + .5 (0.002)^2 qdd_des(t-1)
    # x_02=array([[3.16248706e-06],
    #        [1.99342420e-03],
    #        [9.99999988e-01]])

    ## Parameter identification for target_current

    if False:

        b_1 = np.array(target_current[:,joi]).reshape((-1,1))
        max_current = np.max(target_current[:,joi])
        min_current = np.min(target_current[:,joi])
        thresh = 0.95
        w_1 = (1 - (target_current[:,joi]>thresh*max_current) - (target_current[:,joi]<thresh*min_current)).reshape((-1,1))
        A_1 = np.hstack([#np.ones(b_1.shape), 
            # np.roll(actual_q[:,[joi]],1,axis=0),  
            # np.roll(actual_qd[:,[joi]],1,axis=0), 
            # np.roll(target_q[:,[joi]],0,axis=0),  
            np.roll(np.clip(target_qd[:,[joi]],-0.01, 0.01),0,axis=0), 
            # np.roll(target_qdd[:,[joi]],0,axis=0)
            np.roll(target_moment[:,[joi]],0,axis=0)
            # actual_qd[:,[joi]]/(np.abs(actual_qd[:,[joi]])+1e-4), 
            # # (e/(np.abs(e)+1e-1)).reshape((-1,1)), 
            # (e/(np.abs(e)+1e-3)).reshape((-1,1)),
            # (software_target_q[:,[joi]]-actual_q[:,[joi]]), 
            # # (software_target_q[:,[joi]]-actual_q[:,[joi]])/(np.abs((software_target_q[:,[joi]]-actual_q[:,[joi]]))+1e-4),
            # # int_e.reshape((-1,1))
            ])
        x_1 = np.linalg.solve(A_1.T@(w_1*A_1), A_1.T@(w_1*b_1))
        resid_1 = w_1*(b_1-A_1@x_1)
        RMSE_1 = float(np.sqrt(resid_1.T@resid_1/(np.sum(w_1)))[0,0])
        print(f"{x_1=}")
        print(f"{RMSE_1=} amps") 
        print(f"{min_current=} {max_current=}")
        Ktau = float(1/x_1[1,0])

    

    # target_current = clamp(-0.245, 0.245, 24.08 qd_des + 0.1185 target_moment)# 8.43 Nm/A, roughly, and empirically it's about 8.6 Nm/A with a simple field test
    # x_1=array([[22.71198699],
    #        [ 0.11852733]])
    # RMSE_1=0.0009797746394819675 amps
    # print(f"{Ktau=} vs {literature_Ktau[joi]=}")

    ## Recreation of internal states
    j = actual_q[:,joi] # likely the output encoder
    emd = actual_qd[:,joi] # likely the commutation encoder

    ## Parameter identification for actual_current
    e = target_q[:,joi]-actual_q[:,joi]


    thresh = 1e7
    int_e = e
    for i in range(1, len(e)):
        int_e[i]=int_e[i-1]+(timestamp[i]-timestamp[i-1])*e[i]
        if (int_e[i]>thresh): int_e[i]=thresh
        if (int_e[i]<-thresh): int_e[i]=-thresh
    b_11 = np.array(actual_current[:,joi]).reshape((-1,1))
    A_11 = np.hstack([
        np.ones(b_11.shape), 
        # 500*(e-np.roll(e, 1)).reshape((-1,1)),
        np.roll(e.reshape((-1,1)),1,axis=0),
        # int_e.reshape((-1,1)),
        target_current[:,[joi]],
        np.roll(target_current[:,[joi]],50,axis=0),
        ])
    x_11 = np.linalg.solve(A_11.T@(A_11), A_11.T@(b_11))
    resid_11 = (b_11-A_11@x_11)
    RMSE_11 = float(np.sqrt(resid_11.T@resid_11/(len(b_11)))[0,0])
    print(f"{x_11=}")
    print(f"{RMSE_11=} amps") 
    # print(f"{min_current=} {max_current=}")


    # print(timestamp)
    # plt.plot(timestamp-timestamp[0], software_target_q[:,0])/
    # plt.title("Joint %d"%joi)

    if True:
        fig, axs = plt.subplots(5,1,sharex=True, num = "Joint %d"%joi)
        axs[0].plot(timestamp-timestamp[0], actual_q[:,joi], label="actual_q")
        axs[0].plot(timestamp-timestamp[0], software_target_q[:,joi], label="software_target_q")
        axs[0].plot(timestamp-timestamp[0], target_q[:,joi], label="target_q")
        # axs[0].plot(timestamp-timestamp[0], A_02@x_02, label="model_target_qd")
        axs[0].plot(timestamp-timestamp[0], ctrl_u[:,joi]+actual_q[:,joi], label="u")
        axs[0].plot(timestamp[10:]-timestamp[0], sim_q[10:,joi], label="sim_q")
        axs[1].plot(timestamp-timestamp[0], actual_qd[:,joi], label="actual_qd")
        axs[1].plot(timestamp-timestamp[0], target_qd[:,joi], label="target_qd")
        axs[1].plot(timestamp-timestamp[0], A_01@x_01, label="model_target_qd")
        # axs[0].plot(timestamp-timestamp[0], actual_joint_voltage[:,joi]-actual_joint_voltage[0,joi], label="actual_joint_voltage")
        axs[2].plot(timestamp-timestamp[0], actual_current[:,joi], label="actual_current")
        # axs[2].plot(timestamp-timestamp[0], A_1@x_1, label="model_target_current")
        # axs[2].plot(timestamp-timestamp[0], A_11@x_11, label="model_current")
        axs[2].plot(timestamp-timestamp[0], target_current[:,joi], label="target_current")
        axs[2].plot(timestamp-timestamp[0], target_moment[:,joi]/literature_Ktau[joi], label="moment_current")
        axs[2].plot(timestamp-timestamp[0], id_torques[:,joi]/literature_Ktau[joi], label="id_current")
        axs[3].plot(timestamp-timestamp[0], target_moment[:,joi], label="target_moment")
        axs[3].plot(timestamp-timestamp[0], id_torques[:,joi], label="id_moment")
        axs[4].plot(timestamp-timestamp[0], target_qdd[:,joi], label="target_qdd")
        axs[4].plot(timestamp-timestamp[0], A_00@x_00, label="model_target_qdd")
        axs[0].legend()
        axs[1].legend()
        axs[2].legend()
        axs[3].legend()
        axs[4].legend()
        axs[4].set_xlabel("time (s)")
        axs[4].set_ylabel("qdd")
        axs[3].set_ylabel("moment")
        axs[2].set_ylabel("current")
        axs[1].set_ylabel("qd")
        axs[0].set_ylabel("q")

    if False:
        fig, axs = plt.subplots(3,1,sharex=True, num = "Actual Current Modeling Joint %d"%joi)
        axs[0].plot(timestamp-timestamp[0], actual_q[:,joi], label="q")
        axs[0].plot(timestamp-timestamp[0], target_q[:,joi], label="q_des")
        axs[1].plot(timestamp-timestamp[0], actual_qd[:,joi], label="qd")
        axs[1].plot(timestamp-timestamp[0], target_qd[:,joi], label="qd_des")
        axs[2].plot(timestamp-timestamp[0], actual_current[:,joi] - target_current[:,joi], label="current diff")
        # axs[2].plot(timestamp-timestamp[0], A_11@x_11, label="model_current")
        # axs[2].plot(timestamp-timestamp[0], , label="target_current")
        axs[0].legend()
        axs[1].legend()
        axs[2].legend()
        axs[-1].set_xlabel("time (s)")
        axs[2].set_ylabel("current")
        axs[1].set_ylabel("qd")
        axs[0].set_ylabel("q")
    # plt.plot(timestamp-timestamp[0], joint_temperatures[:,0])
plt.show()
