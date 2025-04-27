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
    
   
   
class InverseDynamics():
    def __init__(self):
        # This is a model, which we keep around so we can ask it for the mass matrix and gravity
        self.plant_, self.model_idx_ = local_load_in_multibody_plant()
        #self.nstates = self.plant_.num_multibody_states(self.model_idx_)
        #self.nq = self.plant_.num_positions(self.model_idx_)
        #self.nv = self.plant_.num_velocities(self.model_idx_)
        #print(f"{self.nstates=}, {self.nq=}, {self.nv=}")
        self.plant_context = self.plant_.CreateDefaultContext()
    
    def calc_ID_tau(self, q, qd, qdd):
        # update what we think the plant is at
        self.plant_context = self.plant_.CreateDefaultContext()
        sampled_states = np.hstack((q, qd))
        self.plant_.SetPositionsAndVelocities(self.plant_context, sampled_states)
        
        
        # build system matricies
        # recall these are defined like
        # M*dv + C*v = Ut*tau_m + tau_g
        # see below for more info:
        # https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_multibody_plant.html
        self.Cv = self.plant_.CalcBiasTerm(self.plant_context) # calcs C(q, v) v
        self.Vq = self.plant_.CalcGravityGeneralizedForces(self.plant_context)  # calcs tau_g(q)
        self.Mq = self.plant_.CalcMassMatrix(self.plant_context) # calcs M(q)

        # \tau = Mq @ qdd + Vq 
    
        tau_control = self.Mq@qdd + self.Cv - self.Vq  # git the manipulator the torques to hold steady over gravity
        return tau_control

def test_ID():
    idyn = InverseDynamics()
    print(idyn.calc_ID_tau(np.array([0,-2.1,-1.5,-1.5,3.14,-3.14]),np.array([10,0,0,0,0,0]),np.array([0,0,0,0,0,0])))
    print(f"{idyn.Vq.T=} {idyn.Cv.T=}")
    print(idyn.calc_ID_tau(np.array([0,-2.1,-1.5,-1.5,3.14,-3.14]),np.array([100,0,0,0,0,0]),np.array([0,0,0,0,0,0])))
    print(f"{idyn.Vq.T=} {idyn.Cv.T=}")


if __name__=="__main__":
    test_ID()

