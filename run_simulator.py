from pydrake.all import ( 
    AddMultibodyPlant,
    MultibodyPlantConfig,
    DiagramBuilder,
    Parser,
    Meshcat,
    AddDefaultVisualization,
    Simulator,
    LogVectorOutput
)

from drake_controller_wrapper import FeedbackControllerDrakeSimWrapper
import numpy as np
import matplotlib.pyplot as plt





ur_path = "./ur_description/urdf/ur5e.urdf"
ur_pkg = "./ur_description/package.xml"

# set up a diagram and meshcat viewer
builder = DiagramBuilder()
meshcat = Meshcat()

# initialize the plant with the simulation specific parameters
plant, scene_graph = AddMultibodyPlant(
    MultibodyPlantConfig(
        time_step=2e-3 # indicates that the solver should use a 500Hz timestep
        ),builder=builder)
parser = Parser(plant)
parser.package_map().AddPackageXml(ur_pkg)

model_idx = parser.AddModels(ur_path)[0]

# weld the robot to the workspace or else it would just fall
# plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base_link"))
# finish editing the plant
plant.Finalize()

# add our feedback controller block into the system diagram
controller = builder.AddSystem(FeedbackControllerDrakeSimWrapper())

# connect the input and output ports
builder.Connect(controller.GetOutputPort("tau_m"), plant.get_actuation_input_port())
builder.Connect(plant.get_state_output_port(), controller.GetInputPort('robot_state'))

# log some data
state_logger = LogVectorOutput(plant.get_state_output_port(), builder)
control_logger = LogVectorOutput(controller.GetOutputPort("tau_m"), builder)

# connect the meshcat visualizer
AddDefaultVisualization(builder, meshcat)

# Finalize and Build the Diagram
diagram = builder.Build()

diagram_context = diagram.CreateDefaultContext()
robot_context = diagram.GetMutableSubsystemContext(plant, diagram_context)


# simulate the system
simulator = Simulator(diagram)

meshcat.StartRecording()# these will add playback and video record buttons to meshcat
simulator.set_target_realtime_rate(1)
simulator.AdvanceTo(10) # final time to simulate to
meshcat.PublishRecording()

# se up plots
state_log = state_logger.FindLog(simulator.get_context())
contr_log = control_logger.FindLog(simulator.get_context())

state_data = state_log.data().transpose()
contr_data = contr_log.data().transpose()

state_times = state_log.sample_times()
contr_times = contr_log.sample_times()

np.savetxt("state_data.csv", state_data, fmt='%.18e', delimiter=",")
np.savetxt("contr_data.csv", contr_data, fmt='%.18e', delimiter=",")
np.savetxt("state_times.csv", state_times, fmt='%.18e', delimiter=",")
np.savetxt("contr_times.csv", contr_times, fmt='%.18e', delimiter=",")


