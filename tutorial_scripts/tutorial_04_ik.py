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
from pydrake.all import (
    InverseKinematics,
    Solve,
)


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


######################################################################################################
#                             ########IK Solver #######   
######################################################################################################

def solve_ik(plant, context, frame_E, X_WE_desired):
    """
    Solves inverse kinematics for a given end-effector pose.

    Args:
        plant: MultibodyPlant
        context: plant.CreateDefaultContext() or similar
        frame_E: End-effector Frame (e.g. plant.GetFrameByName("ee"))
        X_WE_desired: RigidTransform of desired world pose of end-effector

    Returns:
        q_solution: numpy array of joint positions if successful, else None
    """
    ik = InverseKinematics(plant, context)

    # Set nominal joint positions to current positions
    q_nominal = plant.GetPositions(context).reshape((-1, 1))

    
    # Constrain position and orientation
    # Position constraint
    p_AQ = X_WE_desired.translation().reshape((3, 1))
    ik.AddPositionConstraint(
        frameB=frame_E,
        p_BQ=np.zeros((3, 1)),  # Here, p_BQ = [0, 0, 0] means we’re constraining the origin of the E frame.
        frameA=plant.world_frame(),
        p_AQ_lower=p_AQ,
        p_AQ_upper=p_AQ
    )

    # Orientation constraint
    theta_bound = 1e-2  # radians
    ik.AddOrientationConstraint(
        frameAbar=plant.world_frame(),      # world frame
        R_AbarA=X_WE_desired.rotation(),    # desired orientation
        frameBbar=frame_E,                  # end-effector frame
        R_BbarB=RotationMatrix(),           # current orientation
        theta_bound=theta_bound             # allowable deviation
    )

    # Access the underlying MathematicalProgram to add costs and constraints manually.
    prog = ik.prog()
    q_var = ik.q()  # decision variables (joint angles)
    # Add a quadratic cost to stay close to the nominal configuration:
    #   cost = (q - q_nominal)^T * W * (q - q_nominal)
    W = np.identity(q_nominal.shape[0])
    prog.AddQuadraticErrorCost(W, q_nominal, q_var)

    # Enforce joint position limits from the robot model.
    lower = plant.GetPositionLowerLimits()
    upper = plant.GetPositionUpperLimits()
    prog.AddBoundingBoxConstraint(lower, upper, q_var)


    # Solve the optimization problem using Drake’s default solver.
    # The initial guess is the nominal configuration (q_nominal).
    result = Solve(prog, q_nominal)

    # Check if the solver succeeded and return the solution.
    if result.is_success():
        q_sol = result.GetSolution(q_var)
        return q_sol
    else:
        print("IK did not converge!")
        return None

######################################################################################################

# Function to Create Simulation Scene
def create_sim_scene(sim_time_step):
    # Create a DiagramBuilder to construct the system diagram. 
    builder = DiagramBuilder()
    # Add a MultibodyPlant and a SceneGraph to the diagram.
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=sim_time_step)
    # Create a parser to load robot models
    parser = Parser(plant)
    # Load the robot model from the specified URDF file
    parser.AddModelsFromUrl("file://" + os.path.abspath(robot_path))
    # Access the base link (root body) of the Panda robot.
    base_link = plant.GetBodyByName("panda_link0")
    # Weld (fix) the robot’s base frame to the world frame.
    # This prevents the robot from moving as a free-floating body.
    plant.WeldFrames(plant.world_frame(), base_link.body_frame())
    # Finalize the plant setup.
    # After this step, you can no longer add new bodies or joints.
    # The plant is now ready for simulation or controller connections.
    plant.Finalize()

    # Set the initial joint position of the robot otherwise it will correspond to zero positions
    plant.SetDefaultPositions([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.0, 0.0])
    print(plant.GetDefaultPositions())
    # Create a default Context for the MultibodyPlant.
    # The Context stores the current state of the system (positions, velocities, forces, etc.)
    # and provides the workspace where simulation and control computations happen.
    context = plant.CreateDefaultContext()
    # The panda_hand is the end-effector frame
    frame_E = plant.GetFrameByName("panda_hand")
    
    X_WE_desired = RigidTransform(
        RollPitchYaw(np.pi, 0, 0),
        [0.6, 0.0, 0.4]
    )
    # Solve inverse kinematics to find target joint positions for the desired end-effector pose
    q_target = solve_ik(plant, context, frame_E, X_WE_desired)
    # Add visualization to see the geometries in MeshCat
    AddDefaultVisualization(builder=builder, meshcat=meshcat)

    # Add a PD+G controller to regulate the robot
    controller = builder.AddNamedSystem("PD+G controller", Controller(plant))
    
    # Create a constant source for desired positions
    des_pos_system = builder.AddNamedSystem("Desired position", ConstantVectorSource(q_target))
    
    # Connect systems: plant outputs to controller inputs, and vice versa
    builder.Connect(plant.get_state_output_port(), controller.GetInputPort("Current_state")) 
    builder.Connect(controller.GetOutputPort("tau_u"), plant.GetInputPort("applied_generalized_force"))
    builder.Connect(des_pos_system.get_output_port(), controller.GetInputPort("Desired_state"))

    # Create a logger to record the robot's state over time.
    # 'plant.get_state_output_port()' outputs all state variables (positions + velocities).
    # LogVectorOutput connects this output to a logger system so we can access data later.
    logger_state = LogVectorOutput(plant.get_state_output_port(), builder)
    # Give the logger a descriptive name (useful for visualization or debugging).
    logger_state.set_name("State logger")

    # Build the complete system diagram.
    # This finalizes all connections between components (plant, scene graph, logger, etc.)
    # and returns a single Diagram that can be simulated.
    diagram = builder.Build()
    return diagram, logger_state, q_target

# Create a function to run the simulation scene and save the block diagram:
def run_simulation(sim_time_step):
    diagram, logger_state, q_target = create_sim_scene(sim_time_step)
    # Initialize the Drake simulator with the built diagram.
    # The simulator handles time stepping, integration, and advancing system dynamics.
    simulator = Simulator(diagram)
    # Get a mutable reference to the simulator's context.
    # This context stores all runtime variables (states, parameters, etc.)
    # and can be modified before or during simulation (e.g., to set initial states).
    simulator_context = simulator.get_mutable_context()
    # Prepare the simulator (allocate integrators, initialize internal states, etc.)
    simulator.Initialize()
    # Set the desired real-time rate.
    # A rate of 1.0 means "try to run in real time" (simulation time ≈ wall-clock time).
    # You can set higher or lower values to speed up or slow down simulation playback.
    simulator.set_target_realtime_rate(1.)

    maybe_save_block_diagram(diagram, "figures/block_diagram_04_ik.png")
    
    # Run simulation and record for replays in MeshCat
    meshcat.StartRecording()
    simulator.AdvanceTo(5.0)  # Adjust this time as needed
    meshcat.PublishRecording()

    # At the end of the simulation
    plot_transient_response(logger_state, simulator.get_context(), q_target)

# Run the simulation with a specific time step. Try gradually increasing it!
run_simulation(sim_time_step=0.01)
