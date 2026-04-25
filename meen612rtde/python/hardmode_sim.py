from pydrake.all import ( 
    AddMultibodyPlant,
    MultibodyPlantConfig,
    DiagramBuilder,
    Parser,
    Meshcat,
    AddDefaultVisualization,
    Simulator,
    LogVectorOutput,
    ZeroOrderHold,
    DiscreteTimeDelay
)
from pydrake.multibody.math import SpatialForce
from pydrake.multibody.plant import ExternallyAppliedSpatialForce
from pydrake.systems.primitives import ConstantValueSource
from pydrake.common.value import Value

from drake_controller_wrapper import FeedbackControllerDrakeSimWrapper
import numpy as np
import matplotlib.pyplot as plt





ur_path = "./ur_description/urdf/ur5e.urdf"
ur_pkg = "./ur_description/package.xml"

# set up a diagram and meshcat viewer
builder = DiagramBuilder()
meshcat = Meshcat()

sim_timestep = 2e-4 # 5000 Hz
ZOH_timestep = 2e-3 # 500 Hz

# initialize the plant with the simulation specific parameters
plant, scene_graph = AddMultibodyPlant(
    MultibodyPlantConfig(
        time_step=sim_timestep # indicates that the solver should use a 500Hz timestep
        ),builder=builder)
parser = Parser(plant)
parser.package_map().AddPackageXml(ur_pkg)

model_idx = parser.AddModels(ur_path)[0]

initial_q = np.array([0.162774, -1.39593, -1.44978, -1.89947, 1.71143, 3.4145])
initial_qd = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

# weld the robot to the workspace or else it would just fall
# plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base_link"))
# finish editing the plant
plant.Finalize()


# add our feedback controller block into the system diagram
controller = builder.AddSystem(FeedbackControllerDrakeSimWrapper())
# torque_ZOH = builder.AddSystem(ZeroOrderHold(period_sec=ZOH_timestep, vector_size=6))
N_delay = 10
torque_ZOH = builder.AddSystem(DiscreteTimeDelay(update_sec=ZOH_timestep, delay_time_steps = N_delay, vector_size=6))
# torque_delay = builder.
measurement_ZOH = builder.AddSystem(ZeroOrderHold(period_sec=ZOH_timestep, vector_size=12))

external_force = ExternallyAppliedSpatialForce()
external_force.body_index = plant.GetBodyByName("tool0").index()
external_force.F_Bq_W = SpatialForce(tau=[0.0, 0.0, 0.0], f=[-1.0, 0.0, 0.0])
external_forces = builder.AddSystem(ConstantValueSource(Value([external_force])))
external_forces.name="External Force on Tool"

# connect the input and output ports
builder.Connect(controller.GetOutputPort("tau_m"), torque_ZOH.get_input_port())
builder.Connect(torque_ZOH.get_output_port(), plant.get_actuation_input_port())
builder.Connect(plant.get_state_output_port(), measurement_ZOH.get_input_port())
builder.Connect(measurement_ZOH.get_output_port(), controller.GetInputPort('robot_state'))
builder.Connect(external_forces.get_output_port(), plant.get_applied_spatial_force_input_port())

# log some data
state_logger = LogVectorOutput(measurement_ZOH.get_output_port(), builder)
control_logger = LogVectorOutput(torque_ZOH.get_output_port(), builder)

# connect the meshcat visualizer
AddDefaultVisualization(builder, meshcat)

# Finalize and Build the Diagram
diagram = builder.Build()

diagram_context = diagram.CreateDefaultContext()
robot_context = diagram.GetMutableSubsystemContext(plant, diagram_context)

# simulate the system
simulator = Simulator(diagram)
# print(dir(simulator.get_mutable_context()))

## Initial conditions
robot_context = diagram.GetMutableSubsystemContext(plant, simulator.get_context())
plant.SetPositions(robot_context, initial_q)
plant.SetVelocities(robot_context, initial_qd)

# The initial value of the delayed torque should look like the initial output of the controller or more accurately, gravity compensation
initial_tau_control = controller.GetInitialGravity(initial_q, initial_qd)
print(dir(diagram.GetMutableSubsystemContext(torque_ZOH, simulator.get_context())))
diagram.GetMutableSubsystemContext(torque_ZOH, simulator.get_context()).SetDiscreteState(np.array([initial_tau_control]*(N_delay+1)).reshape((-1,)))

# The initial value of the measurement ZOH should look like the real robot state.
diagram.GetMutableSubsystemContext(measurement_ZOH, simulator.get_context()).SetDiscreteState(np.array([initial_q,initial_qd]).reshape((-1,)))

# exit()

## Simulation

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


