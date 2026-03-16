import os
import shutil
import numpy as np
import pydot
from IPython.display import SVG, display
import matplotlib.pyplot as plt
from pydrake.common import temp_directory
from pydrake.geometry import StartMeshcat
from pydrake.math import RotationMatrix, RigidTransform, RollPitchYaw
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.visualization import AddDefaultVisualization
from pydrake.systems.framework import LeafSystem
from pydrake.systems.primitives import ConstantVectorSource, LogVectorOutput
from pydrake.all import Variable, MakeVectorVariable

from helper.dynamics import CalcRobotDynamics


def maybe_save_block_diagram(diagram, image_path):
    """Save the diagram image when Graphviz is installed."""
    if shutil.which("dot") is None:
        print("Skipping block diagram export because Graphviz 'dot' was not found in PATH.")
        return

    svg_data = diagram.GetGraphvizString(max_depth=2)
    graph = pydot.graph_from_dot_data(svg_data)[0]
    graph.write_png(image_path)
    print(f"Block diagram saved as: {image_path}")

# Start the visualizer and clean up previous instances
meshcat = StartMeshcat()
meshcat.Delete()
meshcat.DeleteAddedControls()

# Set the path to your robot model:
robot_path = os.path.join(
    "..", "models", "descriptions", "robots", "arms", "franka_description", "urdf", "panda_arm_hand.urdf"
)

def plot_transient_response(logger_state, simulator_context, desired_positions, num_joints=9):
    """
    Plot joint positions (first 7 joints) and velocities from a LogVectorOutput logger.
    Shows desired positions (q_r) vs actual positions (q_n).

    Parameters
    ----------
    logger_state : LogVectorOutput
        The logger that recorded the system state.
    simulator_context : Context
        The simulator context to extract logged data.
    desired_positions : list or np.array
        Desired joint positions (q_r) for plotting.
    num_joints : int
        Number of joints in the robot (default=9).
    """
    log = logger_state.FindLog(simulator_context)
    time = log.sample_times()        # shape: (num_samples,)
    data = log.data()                # shape: (num_joints*2, num_samples)

    # Separate joint positions and velocities
    q = data[:num_joints, :]        # actual positions
    qdot = data[num_joints:, :]     # velocities

    # Convert desired_positions to array if needed
    q_r = np.array(desired_positions)

    # ---------------- Joint Positions (first 7) ----------------
    fig, axes = plt.subplots(7, 1, figsize=(12, 14), sharex=True)
    for i in range(7):
        axes[i].plot(time, q[i, :], label='q_n')           # actual
        axes[i].plot(time, q_r[i]*np.ones_like(time), '--', label='q_r')  # desired
        axes[i].set_ylabel(f'Joint {i+1} [rad]')
        axes[i].legend()
        axes[i].grid(True)
    axes[-1].set_xlabel('Time [s]')
    fig.suptitle('Joint Positions vs Desired Positions (q_r vs q_n)')
    plt.tight_layout(rect=[0, 0, 1, 0.97])
    plt.show()

    # ---------------- Joint Velocities (first 7) ----------------
    fig, axes = plt.subplots(7, 1, figsize=(12, 14), sharex=True)
    for i in range(7):
        axes[i].plot(time, qdot[i, :], label=f'Joint {i+1} velocity')
        axes[i].set_ylabel(f'Joint {i+1} [rad/s]')
        axes[i].legend()
        axes[i].grid(True)
    axes[-1].set_xlabel('Time [s]')
    fig.suptitle('Joint Velocities')
    plt.tight_layout(rect=[0, 0, 1, 0.97])
    plt.show()

######################################################################################################
#                             ########Define PD+G Controller as a LeafSystem #######   
######################################################################################################

class Controller(LeafSystem):
    def __init__(self, plant):
        super().__init__()

        # Declare input ports for desired and current states
        self._current_state_port = self.DeclareVectorInputPort(name="Current_state", size=18)
        self._desired_state_port = self.DeclareVectorInputPort(name="Desired_state", size=9)

        # PD+G gains (Kp and Kd)
        self.Kp_ = [120.0, 120.0, 120.0, 100.0, 50.0, 45.0, 15.0, 120, 120]
        self.Kd_ = [8.0, 8.0, 8.0, 5.0, 2.0, 2.0, 2.0, 5, 5]

        # Store plant and context for dynamics calculations
        self.plant, self.plant_context_ad = plant, plant.CreateDefaultContext()

        # Declare discrete state and output port for control input (tau_u)
        state_index = self.DeclareDiscreteState(9)  # 9 state variables.
        self.DeclareStateOutputPort("tau_u", state_index)  # output: y=x.
        self.DeclarePeriodicDiscreteUpdateEvent(
            period_sec=1/1000,  # One millisecond time step.
            offset_sec=0.0,  # The first event is at time zero.
            update=self.compute_tau_u) # Call the Update method defined below.
        
        # Create a periodic event to publish the Dynamics of our robot.
        self.DeclarePeriodicPublishEvent(1, 0, self.PublishDynamics) 
    
    def compute_tau_u(self, context, discrete_state):
        num_positions = self.plant.num_positions()
        num_velocities = self.plant.num_velocities()

        # Evaluate the input ports
        self.q_d = self._desired_state_port.Eval(context)
        self.q = self._current_state_port.Eval(context)

        # Compute gravity forces for the current state
        self.plant_context_ad.SetDiscreteState(self.q)
        gravity = -self.plant.CalcGravityGeneralizedForces(self.plant_context_ad)      
        
        tau = self.Kp_ * (self.q_d - self.q[:num_positions]) - self.Kd_ * self.q[num_positions:] + gravity

        # Update the output port = state
        discrete_state.get_mutable_vector().SetFromVector(tau)

    def PublishDynamics(self, context, mode='numerical'):
        print("Publishing event")

        # Get current state
        current_state = self._current_state_port.Eval(context)
        q = current_state[:9]
        qdot = current_state[9:]

        if mode == 'numerical':
            # Evaluate the dynamics numerically
            (M, Cv, tauG, B, tauExt) = CalcRobotDynamics(self.plant, q=q, v=qdot)
        elif mode == 'symbolic':
            # Evaluate the dynamics symbolically
            # Symbolic variables for joint positions and velocities
            self.q_sym = MakeVectorVariable(9, "q")
            self.qdot_sym = MakeVectorVariable(9, "qdot")
            (M, Cv, tauG, B, tauExt) = CalcRobotDynamics(self.plant.ToSymbolic(), q=self.q_sym, v=self.qdot_sym)
        else:
            raise ValueError("Invalid mode. Choose 'numerical' or 'symbolic'.")

        print("M = \n" + str(M))
        print("Cv = " + str(Cv))
        print("tau_G = " + str(tauG))
        print("B = " + str(B))
        print("tau_ext = " + str(tauExt))

# Function to Create Simulation Scene
def create_sim_scene(sim_time_step):   
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=sim_time_step)
    parser = Parser(plant)
    parser.AddModelsFromUrl("file://" + os.path.abspath(robot_path))
    base_link = plant.GetBodyByName("panda_link0")  # replace with your robot’s root link name
    plant.WeldFrames(plant.world_frame(), base_link.body_frame())
    plant.Finalize()

    # Set the initial joint position of the robot otherwise it will correspond to zero positions
    plant.SetDefaultPositions([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.0, 0.0])
    print(plant.GetDefaultPositions())

    # Add visualization to see the geometries in MeshCat
    AddDefaultVisualization(builder=builder, meshcat=meshcat)

    # Add a PD+G controller to regulate the robot
    controller = builder.AddNamedSystem("PD+G controller", Controller(plant))
    
    # Create a constant source for desired positions
    despos_ = [-0.2, -1.5, 0.2, -2.356, 0.0, 1.571, 0.785, 0.0, 0.0]
    des_pos = builder.AddNamedSystem("Desired position", ConstantVectorSource(despos_))
    
    # Connect systems: plant outputs to controller inputs, and vice versa
    builder.Connect(plant.get_state_output_port(), controller.GetInputPort("Current_state")) 
    builder.Connect(controller.GetOutputPort("tau_u"), plant.GetInputPort("applied_generalized_force"))
    builder.Connect(des_pos.get_output_port(), controller.GetInputPort("Desired_state"))

    logger_state = LogVectorOutput(plant.get_state_output_port(), builder)
    logger_state.set_name("State logger")

    # Build and return the diagram
    diagram = builder.Build()
    return diagram, logger_state

# Create a function to run the simulation scene and save the block diagram:
def run_simulation(sim_time_step):
    diagram, logger_state = create_sim_scene(sim_time_step)
    simulator = Simulator(diagram)
    simulator_context = simulator.get_mutable_context()
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.)

    maybe_save_block_diagram(diagram, "figures/block_diagram_03.png")
    
    # Run simulation and record for replays in MeshCat
    meshcat.StartRecording()
    simulator.AdvanceTo(5.0)  # Adjust this time as needed
    meshcat.PublishRecording()

    # At the end of the simulation
    desired_positions = [-0.2, -1.5, 0.2, -2.356, 0.0, 1.571, 0.785, 0.0, 0.0]
    plot_transient_response(logger_state, simulator.get_context(), desired_positions)

# Run the simulation with a specific time step. Try gradually increasing it!
run_simulation(sim_time_step=0.01)
