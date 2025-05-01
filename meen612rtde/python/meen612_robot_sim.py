from pydrake.all import (
    DiagramBuilder,
    MultibodyPlantConfig,
    Parser,
    AddMultibodyPlant
)
import copy
import numpy as np
from collections import deque

def local_load_in_multibody_plant():   
    ur_path = "./ur_description/urdf/ur5e.urdf"
    ur_pkg = "./ur_description/package.xml"
    # set up a diagram and meshcat viewer
    local_builder = DiagramBuilder()

    # initialize the plant with the simulation specific parameters
    local_plant, _ = AddMultibodyPlant(
        MultibodyPlantConfig(
            time_step=2e-3 # indicates that the solver should use a 500Hz timestep
            ),builder=local_builder)
    parser = Parser(local_plant)
    parser.package_map().AddPackageXml(ur_pkg)
    local_model_idx = parser.AddModels(ur_path)[0]
    # weld the robot to the workspace or else it would just fall
    # local_plant.WeldFrames(local_plant.world_frame(), local_plant.GetFrameByName("base_link"))
    # finish editing the plant
    local_plant.Finalize()

    return local_plant, local_model_idx
    
   
   
class RobotSim():
    def __init__(self, q_0, qd_0, N_delay=0):
        # This is a model, which we keep around so we can ask it for the mass matrix and gravity
        self.plant_, self.model_idx_ = local_load_in_multibody_plant()
        #self.nstates = self.plant_.num_multibody_states(self.model_idx_)
        #self.nq = self.plant_.num_positions(self.model_idx_)
        #self.nv = self.plant_.num_velocities(self.model_idx_)
        #print(f"{self.nstates=}, {self.nq=}, {self.nv=}")
        self.plant_context = self.plant_.CreateDefaultContext()
        self.q = np.array(q_0).reshape((-1,))
        self.qd = np.array(qd_0).reshape((-1,))

        sampled_states = np.hstack((self.q, self.qd))
        self.plant_.SetPositionsAndVelocities(self.plant_context, sampled_states)
        self.Vq = self.plant_.CalcGravityGeneralizedForces(self.plant_context)  # calcs tau_g(q)
        self.Mq = self.plant_.CalcMassMatrix(self.plant_context) # calcs M(q)
        self.tau_delay = deque([-self.Vq]*N_delay)
        # self.update(np.zeros((6,)),dt=0.0)
    
    def update(self, tau_control, tau_delta=0.0, dt=0.002):
        # update what we think the plant is at

        self.plant_context = self.plant_.CreateDefaultContext()

        sampled_states = np.hstack((self.q, self.qd))
        self.plant_.SetPositionsAndVelocities(self.plant_context, sampled_states)
        
        # build system matricies
        # recall these are defined like
        # M*dv + C*v = Ut*tau_m + tau_g
        # see below for more info:
        # https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_multibody_plant.html
        self.Cv = self.plant_.CalcBiasTerm(self.plant_context) # calcs C(q, v) v
        self.Vq = self.plant_.CalcGravityGeneralizedForces(self.plant_context)  # calcs tau_g(q)
        self.Mq = self.plant_.CalcMassMatrix(self.plant_context) # calcs M(q)

        self.tau_delay.appendleft(tau_control)
        
        qdd = np.linalg.solve(self.Mq, self.tau_delay.pop() + tau_delta - self.Cv + self.Vq) # forward dynamics

        self.q += self.qd*dt + 0.5 * qdd* dt**2
        self.qd += qdd * dt

        return self.q, self.qd, qdd

# Literature values from
# Andrea Raviola *, Roberto Guida, Andrea De Martin, Stefano Pastorelli, Stefano Mauro and Massimo Sorli
# "Effects of Temperature and Mounting Configuration on the Dynamic Parameters Identification of Industrial Robots"
# Robotics 2021, 10, 83. https://doi.org/10.3390/robotics10030083 (MDPI)
# See Table 1 on page 4
literature_G = 101.0
literature_Ktau = np.array([0.1350, 0.1361, 0.1355, 0.0957, 0.0865, 0.0893])*literature_G # A/Nm
current_dist = 0.0
    
def test_sim():
    for z in range(6)[4:5]:
        rob = RobotSim(np.array([0.0]*6), np.array([0.0]*6), N_delay=0)
        dist = np.zeros((6,))
        dist[z]=1
        final_q = np.array([-70, -88, -98, -89, 263, -107])*np.pi/180.
        con = FeedbackController(np.array([0.0]*6)+0.01*dist,final_q+0.01*dist)
        tau = np.array([ 6.55645586e-04, -6.03647332e+01, -1.86301062e+01,  7.28433206e-05, -2.25094836e-05 ,-1.03442620e-16])
        qs = []
        traj_qs = []
        for i in range(10000):
            qs.append(np.array(rob.q))
            tau = con.calc_control_effort(rob.q, rob.qd, i*0.002)
            rob.update(tau, tau_delta= dist * literature_Ktau * (0. if i<8000 else 1.)* current_dist , dt=0.002)
            traj_qs.append(con.q_des)
            if i==100:
                print(tau)
            # print(", ".join(["%.2f"%(q*180/np.pi) for q in rob.q]), f"{0.5*rob.qd.T@rob.Mq@rob.qd=}")
        plt.plot(qs)
        plt.plot(traj_qs, ":")
    plt.show()

    
if __name__=="__main__":
    import matplotlib.pyplot as plt
    from my_controller import FeedbackController
    test_sim()

