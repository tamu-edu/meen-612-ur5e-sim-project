from pydrake.all import (
    LeafSystem,
    DiagramBuilder,
    MultibodyPlantConfig,
    Parser,
    AddMultibodyPlant
)
import copy
import numpy as np
from my_controller import FeedbackController

MAX_CONTROL_EFFORT = 25 # Nm

class FeedbackControllerDrakeSimWrapper(LeafSystem):
    '''
        DESCRIPTION
    '''
    def __init__(self, *args, **kwargs):
        '''DO NO EDIT'''
        # drake specific setups
        LeafSystem.__init__(self)
        self.controller = FeedbackController(*args, **kwargs)
        
        # declare avector input port the size of the robots state
        self.DeclareVectorInputPort("robot_state", 12)
        
        #declare an output port for the controller
        self.DeclareVectorOutputPort("tau_m", 6, self.CalcControlEffort)        
        
        
    def CalcControlEffort(self, simulator_context, output):
        # read in the states
        sampled_states = self.GetInputPort("robot_state").Eval(simulator_context)
        simulator_time = simulator_context.get_time()
        q = np.array(sampled_states)[:6]
        qd = np.array(sampled_states)[6:]
        
        tau_control = self.controller.calc_control_effort(q, qd, simulator_time)

        tau_grav = self.GetInitialGravity(q, qd)


        assert(np.all(np.isfinite(tau_control)))
        tau_control = tau_grav + np.clip(tau_control-tau_grav, -MAX_CONTROL_EFFORT, MAX_CONTROL_EFFORT)

        
        # send the computed torues to the robot
        output.SetFromVector(tau_control)

    def GetInitialGravity(self, q, qd):
        # update what we think the plant is at
        self.controller.plant_context = self.controller.plant_.CreateDefaultContext()
        sampled_states = np.hstack((q, qd))
        self.controller.plant_.SetPositionsAndVelocities(self.controller.plant_context, sampled_states)
        
        Vq = self.controller.plant_.CalcGravityGeneralizedForces(self.controller.plant_context)  # calcs tau_g(q)

        tau_control = - Vq 
        return tau_control
