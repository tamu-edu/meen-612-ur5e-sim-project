from ZmqCppSynch import ZmqBinarySynchB
from SoftRealtimeLoop import SoftRealtimeLoop
from SysID import Chirp
import numpy as np
from math import floor
from meen612_robot_sim import RobotSim
# from meen612_ID import InverseDynamics
from my_controller import FeedbackController
from collections import deque
# import csv

# Literature values from
# Andrea Raviola *, Roberto Guida, Andrea De Martin, Stefano Pastorelli, Stefano Mauro and Massimo Sorli
# "Effects of Temperature and Mounting Configuration on the Dynamic Parameters Identification of Industrial Robots"
# Robotics 2021, 10, 83. https://doi.org/10.3390/robotics10030083 (MDPI)
# See Table 1 on page 4
literature_G = 101.0
literature_Ktau = np.array([0.1350, 0.1361, 0.1355, 0.0957, 0.0865, 0.0893])*literature_G # A/Nm



def main(fil):
    synchB = ZmqBinarySynchB(
    bindport="ipc:///tmp/feeds/31",
    connectport="ipc:///tmp/feeds/30")

    first_q_seen = False
    initial_q = None
    control_out = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
    q_sim = control_out

    final_q = np.array([-70, -88, -98, -89, 263, -107])*np.pi/180.
    initial_dist = np.array([0, 0, 0, 0, 1, 1.])*(np.pi/180.)*0.0

    robot_sim = None #
    con = None#FeedbackController()# = InverseDynamics()

    duration = 120

    chirp = Chirp(start_freq_Hz=.5, end_freq_Hz=20, time=duration, repeat=True)
    envelope = Chirp(start_freq_Hz=.1, end_freq_Hz=.2, time=duration, repeat=True)
    sim_delay = deque()


    tau_filt1 = np.array([0.0]*6)
    tau_filt2 = np.array([0.0]*6)

    tau_filt3 = np.array([0.0]*6)
    tau_filt4 = np.array([0.0]*6)

    experiment_state = 0

    for i,t in enumerate(SoftRealtimeLoop(0.0020, report=True)):

        data_in = synchB.update(np.array(control_out))
        if not (data_in is None):
            fil.write(",".join([str(x) for x in data_in]+[str(x) for x in control_out]+[str(x) for x in q_sim])+ "\n")
            timestamp = data_in[0]
            actual_q = data_in[1:7]
            actual_qd = data_in[7:13]
            actual_current = data_in[13:19]
            actual_joint_voltage = data_in[19:25]
            software_target_q = data_in[25:31]
            target_current = data_in[31:37]
            target_moment = data_in[37:43]
            target_q = data_in[43:49]
            target_qd =data_in[49:55]
            target_qdd = data_in[55:61]

            if robot_sim is None:
                robot_sim = RobotSim(actual_q, actual_qd, N_delay=2)
                sim_delay.appendleft((actual_q, np.zeros((6,)), np.zeros((6,))))
                sim_delay.appendleft((actual_q, np.zeros((6,)), np.zeros((6,))))
            if con is None:
                con = FeedbackController(actual_q+initial_dist, final_q)


            tau_command = con.calc_control_effort(robot_sim.q, robot_sim.qd, t)

            tau_disturbance = -np.linalg.solve(np.diagflat(literature_Ktau),(actual_current-target_current))
            tau_filt1 += 0.1*(tau_disturbance-tau_filt1)
            tau_filt2 += 0.1*(tau_filt1 - tau_filt2)
            # tau_disturbance = tau_disturbance - tau_filt2
            tau_filt3 += 0.75*(tau_disturbance - tau_filt2 - tau_filt3)
            tau_filt4 += 0.75*(tau_filt3 - tau_filt4)
            tau_disturbance = tau_filt4

            tau_disturbance *= 0.0 # safe mode for class demos

            q_sim, qd_sim, qdd_sim = robot_sim.update(tau_command, tau_delta=tau_disturbance, dt=0.002)

            # qdd_des = -0.0442 + 99.7 u(t-2) - 99.7 q_des(t-1) - 3.12 qd_des(t-1)
            # qd_des = 0.002 qdd_des(t-1) + qd_des(t-1) ## Euler integration
            # q_des = q_des(t-1) + 0.002 qd_des(t-1) + .5 (0.002)^2 qdd_des(t-1)

            # qdd_des z**2 = 100 u - 100 q_des z - 3.12 qd_des z
            # qd_des = 0.002/(z-1) qdd_des
            # q_des (z-1) = 0.002 qd_des + 0.5 0.002**2 qdd_des
            # q_des  = 0.002/(z-1) qd_des + 0.5 0.002**2/(z-1) qdd_des


            # qdd_des z**2 = 100 u - (100 q_des + dt qd_des + 0.5 dt**2 qdd_des) - 3.12 (dt qdd_des + qd_des)
            # qdd_des z**2 = 100 (u - q_des z) - 3.12 qd_des z
            # q_des z**2 = q_des z + 0.002 qd_des z + 0.5 0.002**2 qdd_des z

            # qdd_des z = 100 (u/z - q_des) - 3.12 qd_des
            q_des = target_q
            qd_des = target_qd
            qdd_des = target_qdd
            sim_delay.appendleft((q_sim, qd_sim, qdd_sim))
            old_q_sim, old_qd_sim, old_qdd_sim = sim_delay.pop()

            e_trk = (old_q_sim - q_des)
            ed_trk = (old_qd_sim - qd_des)
            edd_trk = (old_qdd_sim - qdd_des)
            # q_des z = q_des + qd_des*dt + 0.5 * qdd_des * dt**2
            # q_sim z = q_sim + qd_sim*dt + 0.5 * qdd_sim * dt**2
            # e_trk z = e_trk + ed_trk*dt + 0.5 * edd_trk * dt**2

            # qd_des z = qd_des + qdd_des * dt  
            # qd_sim z = qd_sim + qdd_sim * dt
            # ed_trk z = ed_trk + edd_trk * dt

            # edd_trk = qdd_sim/zz - qdd_des = qdd_sim/zz - 100 u/z/z - q_des/z - 3.12 qd_des/z

            # edd_trk zz = qdd_sim - qdd_des zz = qdd_sim - 100 (u - q_des z) + 3.12 qd_des z (system)
            # edd_trk zz + A ed_trk + B e_trk = 0 (target)

            # -A ed_trk - B e_trk = qdd_sim - 100 (u - q_des z) + 3.12 qd_des z (control law)
            # u = A/100 ed_trk + B/100 e_trk + qdd_sim/100 + q_des z + 3.12 qd_des z/100
            # u = A/100 ed_trk + B/100 e_trk + qdd_sim/100 + (q_des + qd_des*dt + 0.5 * qdd_des * dt**2) + 3.12 (qd_des + qdd_des * dt)/100

            zeta = .7
            omega_n = 20.0
            A = 2*zeta*omega_n#3.12
            B = omega_n**2#99.7
            dt =0.002

            u = A/99.7* ed_trk + B/99.7 *e_trk + qdd_sim/99.7 + (q_des + qd_des*(dt+ 3.12 /99.7) +  qdd_des *( 0.5 * dt**2 + 3.12 * dt/99.7))




            experiment_state = floor(t/duration)
            if experiment_state>=6:
                break

            target_vec = np.zeros((6,))
            target_vec[experiment_state]+=1.0

            system_id = target_vec*chirp.next(t)*1e-3*(0.5+0.5*envelope.next(t))

            if initial_q is None:
                initial_q = np.array(actual_q)

            if not (initial_q is None):
                control_out = (u-actual_q)#+system_id
                # control_out = 1e-1*(initial_q-actual_q)+system_id
        if i%100==0:
            print(control_out)


   
if __name__ == '__main__':
    with open("robot_data.csv", 'w') as fil:
        main(fil)