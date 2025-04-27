from pydrake.all import (
    DiagramBuilder,
    MultibodyPlantConfig,
    Parser,
    AddMultibodyPlant
)
import copy
import numpy as np

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
    
   
   
class FeedbackController():
    def __init__(self, q0, q1):
        # This is a model, which we keep around so we can ask it for the mass matrix and gravity
        self.plant_, self.model_idx_ = local_load_in_multibody_plant()
        #self.nstates = self.plant_.num_multibody_states(self.model_idx_)
        #self.nq = self.plant_.num_positions(self.model_idx_)
        #self.nv = self.plant_.num_velocities(self.model_idx_)
        #print(f"{self.nstates=}, {self.nq=}, {self.nv=}")
        self.q0 = np.array(q0)
        self.q1 = np.array(q1)
        self.plant_context = self.plant_.CreateDefaultContext()
    
    def calc_control_effort(self, q, qd, time_now):
        # update what we think the plant is at
        self.plant_context = self.plant_.CreateDefaultContext()
        sampled_states = np.hstack((q, qd))
        self.plant_.SetPositionsAndVelocities(self.plant_context, sampled_states)
        
        
        # build system matricies
        # recall these are defined like
        # M*dv + C*v = Ut*tau_m + tau_g
        # see below for more info:
        # https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_multibody_plant.html
        Cv = self.plant_.CalcBiasTerm(self.plant_context) # calcs C(q, v) v
        Vq = self.plant_.CalcGravityGeneralizedForces(self.plant_context)  # calcs tau_g(q)
        Mq = self.plant_.CalcMassMatrix(self.plant_context) # calcs M(q)
        
        if time_now>30:
            time_now = 30.

        q0 = self.q0
        q1 = self.q0
        q2 = self.q1
        q3 = self.q1

        path = lambda s: (1-s)**3 * q0 + s*(1-s)**2*3*q1 + 3*s**2*(1-s)*q2 + s**3 * q3

        path_prime = lambda s: (
            3*(1-s)**2 * (-1) * q0 
            + (1-s)**2*3*q1 + s*(1-s)*2*(-1)*3*q1
            + 3*2*s*(1-s)*q2 + 3*s**2*(-1)*q2
            + 3*s**2 * q3
            )

        path_dprime = lambda s: (
            3*2*(1-s)**1 * (1)*q0
            +3*2*(1-s)*(-1)*q1 + (1-s)*2*(-1)*3*q1 + s*(-1)*2*(-1)*3*q1
            + 3*2*(1-s)*q2 + 3*2*s*(-1)*q2 + 3*2*s*(-1)*q2
            + 3*2*s*q3)

        # s = time_now/30
        # dq_des/s = path_prime
        # dq_des/time_now = dq_des/s*ds/dtime_now
        q_des = path(time_now/30)
        qd_des = path_prime(time_now/30)*1/30
        qdd_des = path_dprime(time_now/30)*1/30*1/30
       
        '''Do your control work here'''

        K = np.array([
            [10,0,0,0,0,0],
            [0,10,0,0,0,0],
            [0,0,10,0,0,0],
            [0,0,0,10,0,0],
            [0,0,0,0,1,0],
            [0,0,0,0,0,0.1],
            ])

        freq = 0.2 # Hz
        zeta = 1. # critical damping
        omega_n = 2*np.pi*freq
        Kprime = np.eye(6)*(omega_n)**2
        Bprime = np.eye(6)*2*zeta*(omega_n)


        # a bad controller with oscillating torques about the gravity compensation
        tau_control = Mq@qdd_des + Mq@(- Kprime @ (q-q_des) - Bprime @ (qd-qd_des)) - Vq + Cv 
        return tau_control

def test_controller():
    controller = FeedbackController()
    controller.calc_control_effort(np.array([0,0,0,0,0,0]),np.array([0,0,0,0,0,0]), 0)
    print(dir(controller.plant_))
    #'CalcJacobianAngularVelocity', 'CalcJacobianCenterOfMassTranslationalVelocity', 'CalcJacobianPositionVector', 'CalcJacobianSpatialVelocity', 'CalcJacobianTranslationalVelocity'
    print(controller.plant_.CalcJacobianPositionVector.__doc__)
    
    print(controller.plant_.GetFrameByName("world"))
    world_frame = controller.plant_.GetFrameByName("world")
    try:
        print(controller.plant_.GetFrameByName("invalid_guess_for_frame_name"))
    except RuntimeError as e:
        print(e)
    print(controller.plant_.GetFrameByName("tool0"))
    tool_frame = controller.plant_.GetFrameByName("tool0")
    print(controller.plant_.CalcJacobianPositionVector(
        controller.plant_context, tool_frame, np.array([[0,0,0]]).T, world_frame, world_frame))
    J = controller.plant_.CalcJacobianPositionVector(
        controller.plant_context, tool_frame, np.array([[0,0,0]]).T, world_frame, world_frame)
        
def test_jacobian():  
    controller = FeedbackController()
    q_0 = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
    qd_0 = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
    controller.calc_control_effort(q_0, qd_0, 0.0)
    
    world_frame = controller.plant_.GetFrameByName("world")
    tool_frame = controller.plant_.GetFrameByName("tool0")
    J_hyp = controller.plant_.CalcJacobianPositionVector(
        controller.plant_context, tool_frame, np.array([[0,0,0]]).T, world_frame, world_frame)
    J_hyp_short = J_hyp[:,:3]
    print(J_hyp_short)
    print(f"{np.linalg.svd(J_hyp_short)=}")
    
    scale = 1e-6
    delta_x_desired = np.array([1, 0, 0])*scale
    delta_q_desired = np.zeros((6,))
    delta_q_desired[:3] = np.linalg.solve(J_hyp_short, delta_x_desired)
    # but how do we find the position of the end-effector in the world frame?
    print(dir(controller.plant_))
    print(controller.plant_)
    print(type(controller.plant_))
    print(delta_q_desired)
    print(dir(tool_frame))
    print(tool_frame.CalcPoseInWorld.__doc__)
    print(dir(tool_frame.CalcPoseInWorld(controller.plant_context)))
    x_0 = tool_frame.CalcPoseInWorld(controller.plant_context
     ).translation()
    print(tool_frame.CalcPoseInWorld(controller.plant_context
     ).translation())
     
    q_1 = q_0 + delta_q_desired
    
    controller.calc_control_effort(q_1, qd_0, 0.0)
    
    x_1 = tool_frame.CalcPoseInWorld(controller.plant_context
     ).translation()
     
    print(f"{(x_1-x_0)/scale=}, {delta_x_desired/scale=}")
    

if __name__=="__main__":
    #test_controller()
    test_jacobian()
