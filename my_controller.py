from pydrake.all import LeafSystem
import copy
import numpy as np

class FeedbackController(LeafSystem):
    '''
        DESCRIPTION
    '''
    def __init__(self, plant, model_idx):
       
        # drake specific setups
        LeafSystem.__init__(self)
        self.nstates = plant.num_multibody_states(model_idx)
        self.nq = plant.num_positions(model_idx)
        self.nv = plant.num_velocities(model_idx)

        # declare avector input port the size of the robots state
        self.DeclareVectorInputPort("robot_state", self.nstates)
        
        #declare an output port for the controller
        self.DeclareVectorOutputPort("tau_m", plant.num_actuated_dofs(model_idx), self.CalcControlEffort)
        self.plant_ = copy.copy(plant)
        self.model_idx_ = model_idx
        
        
    def CalcControlEffort(self, context, output):
        # read in the states
        sampled_states = self.GetInputPort("robot_state").Eval(context)
        
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
       
        time = context.get_time()
        '''Do your control work here'''

        tau_control = np.ones_like(Vq)*np.sin(time)*0.1 - Vq  # git the manipulator the torques to hold steady over gravity
        
        # send the computed torues to the robot
        output.SetFromVector(tau_control)