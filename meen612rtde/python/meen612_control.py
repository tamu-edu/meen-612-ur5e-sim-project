from ZmqCppSynch import ZmqBinarySynchB
from SoftRealtimeLoop import SoftRealtimeLoop
from SysID import Chirp
import numpy as np
from math import floor
from meen612_robot_sim import RobotSim
# from meen612_ID import InverseDynamics
from my_controller import FeedbackController
from collections import deque
from drake_controller_wrapper import FeedbackControllerDrakeSimWrapper, MAX_CONTROL_EFFORT
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

    grav_comp_model = FeedbackControllerDrakeSimWrapper()

    tau_command = np.array([0.0,0.0,0.0,0.0,0.0,0.0])

    # an initial bias to allow observing control transients
    initial_dist = np.array([0, 0, 0, 0, 1, 1.])*(np.pi/180.)*0.0 

    robot_sim = None #
    con = None#FeedbackController()# = InverseDynamics()

    duration = 10

    experiment_state = 0

    for i,t in enumerate(SoftRealtimeLoop(0.0020, report=True)):

        data_in = synchB.update(np.array(tau_command))
        if not (data_in is None):
            # fil.write(",".join([str(x) for x in data_in]+[str(x) for x in control_out]+[str(x) for x in q_sim])+ "\n")
            fil.write(",".join([str(x) for x in data_in]+[str(x) for x in tau_command])+ "\n")
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

            if con is None:
                con = FeedbackController(q0=actual_q+initial_dist)


            tau_command = con.calc_control_effort(actual_q, actual_qd, t)

            tau_grav = grav_comp_model.GetInitialGravity(actual_q, actual_qd)
            assert(np.all(np.isfinite(tau_command)))
            assert(np.all(np.isfinite(tau_grav)))
            # (GRAVITY COMP IS TAKEN CARE OF BY THE ROBOT!)
            tau_command = np.clip(tau_command-tau_grav, -MAX_CONTROL_EFFORT, MAX_CONTROL_EFFORT) # + tau_grav (GRAVITY COMP IS TAKEN CARE OF BY THE ROBOT!)

        if i%100==0:
            print(f"{tau_command=}")

   
if __name__ == '__main__':
    with open("robot_data.csv", 'w') as fil:
        main(fil)