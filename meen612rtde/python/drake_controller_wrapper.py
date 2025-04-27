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

class FeedbackControllerDrakeSimWrapper(LeafSystem):
    '''
        DESCRIPTION
    '''
    def __init__(self):
        '''DO NO EDIT'''
        # drake specific setups
        LeafSystem.__init__(self)
        self.controller = FeedbackController(np.array([-70, -88, -98, -89, 263, -107])*np.pi/180., np.array([-100, -70, -70, -89, 263, -107])*np.pi/180.)
        
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
        
        # send the computed torues to the robot
        output.SetFromVector(tau_control)
