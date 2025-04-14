from pydrake.all import (
    LeafSystem,
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


class FeedbackController(LeafSystem):
    '''
        DESCRIPTION
    '''
    def __init__(self):
        '''DO NO EDIT'''
        # drake specific setups
        LeafSystem.__init__(self)
        self.plant_, self.model_idx_ = local_load_in_multibody_plant()
        self.nstates = self.plant_.num_multibody_states(self.model_idx_)
        self.nq = self.plant_.num_positions(self.model_idx_)
        self.nv = self.plant_.num_velocities(self.model_idx_)

        # declare avector input port the size of the robots state
        self.DeclareVectorInputPort("robot_state", self.nstates)
        
        #declare an output port for the controller
        self.DeclareVectorOutputPort("tau_m", self.plant_.num_actuated_dofs(self.model_idx_), self.CalcControlEffort)
        
        
        
    def CalcControlEffort(self, simulator_context, output):
        # read in the states
        sampled_states = self.GetInputPort("robot_state").Eval(simulator_context)
        
        # update what we think the plant is at
        plant_context = self.plant_.CreateDefaultContext()
        self.plant_.SetPositionsAndVelocities(plant_context, sampled_states)
        
        # build system matricies
        # recall these are defined like
        # M*dv + C*v = Ut*tau_m + tau_g
        # see below for more info:
        # https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_multibody_plant.html
        Cv = self.plant_.CalcBiasTerm(plant_context) # calcs C(q, v) v
        Vq =  self.plant_.CalcGravityGeneralizedForces(plant_context)  # calcs tau_g(q)
        Mq = self.plant_.CalcMassMatrix(plant_context) # calcs M(q)
       
        simulator_time = simulator_context.get_time()
        '''Do your control work here'''

        # a bad controller with oscillating torques about the gravity compensation
        tau_control = np.ones_like(Vq)*np.sin(simulator_time)*0.01 - Vq  # git the manipulator the torques to hold steady over gravity
        
        # send the computed torues to the robot
        output.SetFromVector(tau_control)