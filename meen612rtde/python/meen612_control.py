from ZmqCppSynch import ZmqBinarySynchB
from SoftRealtimeLoop import SoftRealtimeLoop
from SysID import Chirp
import numpy as np
from math import floor
from meen612_robot_sim import RobotSim
# from meen612_ID import InverseDynamics
from my_controller import FeedbackController
# import csv

def main(fil):
    synchB = ZmqBinarySynchB(
    bindport="ipc:///tmp/feeds/31",
    connectport="ipc:///tmp/feeds/30")

    first_q_seen = False
    initial_q = None
    control_out = np.array([0.0,0.0,0.0,0.0,0.0,0.0])

    final_q = np.array([-70, -88, -98, -89, 263, -107])*np.pi/180.
    initial_dist = np.array([0, 0, 0, 0, 1, 1.])*np.pi/180.

    robot_sim = None #
    con = None#FeedbackController()# = InverseDynamics()

    duration = 120

    chirp = Chirp(start_freq_Hz=.5, end_freq_Hz=20, time=duration, repeat=True)
    envelope = Chirp(start_freq_Hz=.1, end_freq_Hz=.2, time=duration, repeat=True)

    experiment_state = 0

    for i,t in enumerate(SoftRealtimeLoop(0.0020, report=True)):

        data_in = synchB.update(np.array(control_out))
        if not (data_in is None):
            fil.write(",".join([str(x) for x in data_in])+ "\n")
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
            if con is None:
                con = FeedbackController(actual_q+initial_dist, final_q)


            tau_command = con.calc_control_effort(robot_sim.q, robot_sim.qd, t)
            q, qd, qdd = robot_sim.update(tau_command, dt=0.002)

            experiment_state = floor(t/duration)
            if experiment_state>=6:
                break

            target_vec = np.zeros((6,))
            target_vec[experiment_state]+=1.0

            system_id = target_vec*chirp.next(t)*1e-3*(0.5+0.5*envelope.next(t))

            if initial_q is None:
                initial_q = np.array(actual_q)

            if not (initial_q is None):
                control_out = (q-actual_q)#+system_id
                # control_out = 1e-1*(initial_q-actual_q)+system_id
        if i%100==0:
            print(control_out)


   
if __name__ == '__main__':
    with open("robot_data.csv", 'w') as fil:
        main(fil)