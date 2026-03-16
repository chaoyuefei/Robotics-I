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
from pydrake.systems.framework import LeafSystem, BasicVector
from pydrake.trajectories import PiecewisePolynomial


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

def plot_joint_tracking(logger_state, logger_traj, simulator_context, num_joints=9):
    """
    Plot actual vs reference joint positions and velocities from logs.
    """
    log_state = logger_state.FindLog(simulator_context)
    log_traj = logger_traj.FindLog(simulator_context)

    time = log_state.sample_times()
    q_actual = log_state.data()[:num_joints, :]
    qdot_actual = log_state.data()[num_joints:, :]

    q_ref = log_traj.data()[:num_joints, :]
    qdot_ref = log_traj.data()[num_joints:, :]

    # --- Joint positions ---
    fig, axes = plt.subplots(7, 1, figsize=(12, 14), sharex=True)
    for i in range(7):
        axes[i].plot(time, q_actual[i, :], label='q_actual')
        axes[i].plot(time, q_ref[i, :], '--', label='q_ref')
        axes[i].set_ylabel(f'Joint {i+1} [rad]')
        axes[i].legend()
        axes[i].grid(True)
        axes[i].set_ylim(-2.5, 2.5)
    axes[-1].set_xlabel('Time [s]')
    fig.suptitle('Joint Positions: Actual vs Reference')
    plt.tight_layout(rect=[0, 0, 1, 0.97])
    plt.show()

    # --- Joint velocities ---
    fig, axes = plt.subplots(7, 1, figsize=(12, 14), sharex=True)
    for i in range(7):
        axes[i].plot(time, qdot_actual[i, :], label='qdot_actual')
        axes[i].plot(time, qdot_ref[i, :], '--', label='qdot_ref')
        axes[i].set_ylabel(f'Joint {i+1} [rad/s]')
        axes[i].legend()
        axes[i].grid(True)
        axes[i].set_ylim(-1.0, 1.0)
    axes[-1].set_xlabel('Time [s]')
    fig.suptitle('Joint Velocities: Actual vs Reference')
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
        self._desired_state_port = self.DeclareVectorInputPort(name="Desired_state", size=18)

        # PD+G gains (Kp and Kd)
        self.Kp_ = np.array([120.0, 120.0, 120.0, 100.0, 50.0, 45.0, 15.0, 120, 120])
        self.Kd_ = 3*np.array([8.0, 8.0, 8.0, 5.0, 2.0, 2.0, 2.0, 5, 5])

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
        self.q_d = self._desired_state_port.Eval(context)[0:num_positions]
        self.q_d_dot = self._desired_state_port.Eval(context)[num_positions:]
        self.q = self._current_state_port.Eval(context)

        # Compute gravity forces for the current state
        self.plant_context_ad.SetDiscreteState(self.q)
        gravity = -self.plant.CalcGravityGeneralizedForces(self.plant_context_ad)      
        
        tau = self.Kp_ * (self.q_d - self.q[:num_positions]) + self.Kd_ * (self.q_d_dot - self.q[num_positions:]) + gravity

        # Update the output port = state
        discrete_state.get_mutable_vector().SetFromVector(tau)


######################################################################################################
#                     ########Define Trajectory Generator as a LeafSystem #######   
######################################################################################################

class JointSpaceTrajectorySystem(LeafSystem):
    def __init__(self, q_start, q_goal, v_max, a_max):
        super().__init__()
        self.q_start = np.array(q_start)
        self.q_goal  = np.array(q_goal)
        self.v_max   = np.broadcast_to(v_max, self.q_start.shape)
        self.a_max   = np.broadcast_to(a_max, self.q_start.shape)
        self.n = len(q_start)
        # Precompute motion profiles for each joint.
        # This sets up the timing and kinematic parameters (acceleration time,
        # flat time, total duration, etc.) used later to evaluate q_ref(t) and qd_ref(t).
        self._compute_profiles()
        self.DeclareVectorOutputPort("joint_ref", BasicVector(2*self.n), self._output_reference)

    def _compute_profiles(self):
        """
        Precompute motion parameters for each joint.

        The trapezoidal velocity profile consists of three phases:
        1. Acceleration with constant a_max
        2. Constant velocity at v_max (flat section)
        3. Deceleration with constant -a_max

        However, depending on the motion distance (Δq) and limits (v_max, a_max),
        the profile can take two shapes:

            • **Trapezoidal** → when the joint reaches v_max before decelerating.
            • **Triangular**  → when the joint does not reach v_max; 
                                acceleration and deceleration phases overlap.

        This method computes the timing parameters (t_acc, t_flat, T_total)
        for each joint and ensures all joints are synchronized by using
        the maximum duration among them.
        """
        self.profiles = []
        self.duration = 0.0

        for i in range(self.n):
            q0, qf = self.q_start[i], self.q_goal[i]
            dq = qf - q0
            s = np.sign(dq) if dq != 0 else 1.0
            dq_abs = abs(dq)
            v = self.v_max[i]
            a = self.a_max[i]

            # Acceleration time and peak velocity
            t_acc = v / a if a > 0 else 0.0

            # Check for triangular vs. trapezoidal
            if dq_abs < a * t_acc**2:
                # Triangular (no constant-velocity phase)
                t_acc = np.sqrt(dq_abs / max(a, 1e-9))
                t_flat = 0.0
                v_peak = a * t_acc
            else:
                # Trapezoidal (with constant velocity)
                v_peak = v
                t_flat = (dq_abs - a * t_acc**2) / max(v, 1e-9)

            T_trap = 2 * t_acc + t_flat
            self.duration = max(self.duration, T_trap)

            self.profiles.append(
                {
                    "q0": q0,
                    "dq": dq,
                    "s": s,
                    "t_acc": t_acc,
                    "t_flat": t_flat,
                    "T": T_trap,
                    "a": a,
                    "v_peak": v_peak,
                }
            )

    def _eval_trapezoid(self, t, p):
        """
        Compute joint position and velocity for a trapezoidal velocity profile.

        Each joint moves following a constant-acceleration, constant-velocity,
        constant-deceleration pattern. The parameter 's' ∈ {+1, -1} represents
        the direction of motion (sign of Δq = q_goal - q_start).

        Motion phases and equations:
            1. Acceleration (0 ≤ t < t_acc)
            q = q0 + s·½·a·t²
            q̇ = s·a·t

            2. Constant velocity (t_acc ≤ t < t_acc + t_flat)
            q = q0 + s·[½·a·t_acc² + v_peak·(t - t_acc)]
            q̇ = s·v_peak

            3. Deceleration (t_acc + t_flat ≤ t < T)
            q = q0 + s·[½·a·t_acc² + v_peak·t_flat
                            + v_peak·t_d - ½·a·t_d²]
            q̇ = s·(v_peak - a·t_d)
            where t_d = t - (t_acc + t_flat)

        Total duration:  T = 2·t_acc + t_flat
        """
        # Extract parameters for this joint
        q0, s, a, t_acc, t_flat, T, dq = (
            p["q0"], p["s"], p["a"], p["t_acc"], p["t_flat"], p["T"], p["dq"]
        )

        if t <= 0:
            q, qd = q0, 0.0
        elif t < t_acc:  # accelerating
            q = q0 + s * 0.5 * a * t**2
            qd = s * a * t
        elif t < t_acc + t_flat:  # constant velocity
            q = q0 + s * (0.5 * a * t_acc**2 + p["v_peak"] * (t - t_acc))
            qd = s * p["v_peak"]
        elif t < T:  # decelerating
            td = t - (t_acc + t_flat)
            q = (q0 + s * (0.5 * a * t_acc**2 + p["v_peak"] * t_flat +
                        p["v_peak"] * td - 0.5 * a * td**2))
            qd = s * (p["v_peak"] - a * td)
        else:  # end of motion
            q, qd = q0 + p["dq"], 0.0
        return q, qd
    
    def _output_reference(self, context, output):
        """Compute [q_ref, qd_ref] at current simulation time."""
        t = context.get_time()
        q_ref, qd_ref = np.zeros(self.n), np.zeros(self.n)
        for i, p in enumerate(self.profiles):
            q_ref[i], qd_ref[i] = self._eval_trapezoid(t, p)
        output.SetFromVector(np.hstack([q_ref, qd_ref]))

######################################################################################################

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
    q_start = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.0, 0.0]
    plant.SetDefaultPositions(q_start)
    q_target = [-0.5, -1.5, 0.5, -1.356, 0.5, 0.5, 0.0, 0.0, 0.0]
    print(plant.GetDefaultPositions())


    # Add visualization to see the geometries in MeshCat
    AddDefaultVisualization(builder=builder, meshcat=meshcat)

    # Add a PD+G controller to regulate the robot
    controller = builder.AddNamedSystem("PD+G controller", Controller(plant))
    
    # Create a constant source for desired positions
    traj_system = builder.AddNamedSystem(
        "Trajectory Generator",
        JointSpaceTrajectorySystem(
            q_start=q_start,
            q_goal=q_target,
            v_max=0.2,   # rad/s
            a_max=2.0,    # rad/s²
        )
    )
    # des_pos = builder.AddNamedSystem("Desired position", ConstantVectorSource(q_target))
    
    # Connect systems: plant outputs to controller inputs, and vice versa
    builder.Connect(plant.get_state_output_port(), controller.GetInputPort("Current_state")) 
    builder.Connect(controller.GetOutputPort("tau_u"), plant.GetInputPort("applied_generalized_force"))
    # builder.Connect(des_pos.get_output_port(), controller.GetInputPort("Desired_state"))
    builder.Connect(traj_system.get_output_port(0), controller.GetInputPort("Desired_state"))


    logger_state = LogVectorOutput(plant.get_state_output_port(), builder)
    logger_state.set_name("State logger")

    logger_traj = LogVectorOutput(traj_system.get_output_port(0), builder)
    logger_traj.set_name("Trajectory logger")

    # Build and return the diagram
    diagram = builder.Build()
    return diagram, logger_state, logger_traj, q_target

# Create a function to run the simulation scene and save the block diagram:
def run_simulation(sim_time_step):
    diagram, logger_state, logger_traj, q_target = create_sim_scene(sim_time_step)
    simulator = Simulator(diagram)
    simulator_context = simulator.get_mutable_context()
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.)

    maybe_save_block_diagram(diagram, "figures/block_diagram_04_traj.png")
    
    # Run simulation and record for replays in MeshCat
    meshcat.StartRecording()
    simulator.AdvanceTo(6.0)  # Adjust this time as needed
    meshcat.PublishRecording()

    # At the end of the simulation
    plot_joint_tracking(logger_state, logger_traj, simulator.get_context())

# Run the simulation with a specific time step. Try gradually increasing it!
run_simulation(sim_time_step=0.01)
