from pydrake.all import ( 
    ModelVisualizer,
    AddMultibodyPlant,
    MultibodyPlantConfig,
    DiagramBuilder,
    Parser,
    Meshcat,
    AddDefaultVisualization,
    Simulator
)

from my_controller import FeedbackController
import numpy as np

ur_path = "./ur_description/urdf/ur3e_cylinders_collision.urdf"

# set up a diagram and meshcat viewer
builder = DiagramBuilder()
meshcat = Meshcat()

# initialize the plant with the simulation specific parameters
plant, scene_graph = AddMultibodyPlant(
    MultibodyPlantConfig(
        time_step=2e-3 # indicates that the solver should use a 500Hz timestep
        ),builder=builder)
parser = Parser(plant)
model_idx = parser.AddModels(ur_path)[0]

# weld the robot to the workspace or else it would just fall
plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("ur_base_link"))
# finish editing the plant
plant.Finalize()

# add our feedback controller block into the system diagram
controller = builder.AddSystem(FeedbackController())

# connect the input and output ports
builder.Connect(controller.GetOutputPort("tau_m"), plant.get_actuation_input_port())
builder.Connect(plant.get_state_output_port(), controller.GetInputPort('robot_state'))

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
simulator.AdvanceTo(np.inf) # final time to simulate to
meshcat.PublishRecording()