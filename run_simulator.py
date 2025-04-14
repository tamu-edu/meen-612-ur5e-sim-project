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

from my_controller import FeedbackController
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
controller = builder.AddSystem(FeedbackController())

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

fig, ax = plt.subplots(1,2)
state_names = plant.GetStateNames() # print this out if you're curious, or use pythons '.index()' function to plot specifics
actuator_names = [plant.get_joint_actuator(i).name() for i in plant.GetJointActuatorIndices()]

ax[0].plot(state_times, state_data, label=state_names)
ax[1].plot(contr_times, contr_data, label=actuator_names)
ax[0].legend()
ax[0].grid()
ax[0].set_xlabel('time (s)')
ax[0].set_ylabel('positions or velocities')
ax[1].legend()
ax[1].grid()
ax[1].set_xlabel('time (s)')
ax[1].set_ylabel('commanded torques')

plt.show()