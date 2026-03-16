import os
import shutil
import numpy as np
from ompl import base as ob
from ompl import control as oc
from ompl import geometric as og
import pydot
from IPython.display import SVG, display
import matplotlib.pyplot as plt
from pydrake.common import temp_directory
from pydrake.geometry import StartMeshcat, Box as DrakeBox
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
    InverseKinematics, Solve,
    SpatialInertia, UnitInertia,
    RigidTransform, CoulombFriction
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

    # --- Joint positions ---
    fig, axes = plt.subplots(7, 1, figsize=(12, 14), sharex=True)
    for i in range(7):
        axes[i].plot(time, q_actual[i, :], label='q_actual')
        axes[i].plot(time, q_ref[i, :], '--', label='q_ref')
        axes[i].set_ylabel(f'Joint {i+1} [rad]')
        axes[i].legend()
        axes[i].grid(True)
        axes[i].set_ylim(-3, 3)
    axes[-1].set_xlabel('Time [s]')
    fig.suptitle('Joint Positions: Actual vs Reference')
    plt.tight_layout(rect=[0, 0, 1, 0.97])
    plt.show()

######################################################################################################
#                             ########Define PD+G Controller as a LeafSystem #######   
######################################################################################################

class Controller(LeafSystem):
    def __init__(self, plant, robot):
        super().__init__()

        # Declare input ports for desired and current states
        self._current_state_port = self.DeclareVectorInputPort(name="Current_state", size=18)
        self._desired_state_port = self.DeclareVectorInputPort(name="Desired_state", size=9)

        # PD+G gains (Kp and Kd)
        self.Kp_ = np.array([120.0, 120.0, 120.0, 100.0, 50.0, 45.0, 15.0, 120, 120])
        self.Kd_ = np.array([8.0, 8.0, 8.0, 5.0, 2.0, 2.0, 2.0, 5, 5])
        self.robot = robot

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
        num_positions = self.plant.num_positions(self.robot)
        num_velocities = self.plant.num_velocities(self.robot)

        # Evaluate the input ports
        self.q_d = self._desired_state_port.Eval(context)[0:num_positions]
        self.q = self._current_state_port.Eval(context)

        # Compute gravity forces for the current state
        self.plant.SetPositionsAndVelocities(self.plant_context_ad, self.robot, self.q)
        gravity = -self.plant.CalcGravityGeneralizedForces(self.plant_context_ad)[:num_positions]
        
        tau = self.Kp_ * (self.q_d - self.q[:num_positions]) - self.Kd_ * self.q[num_positions:] + gravity

        # Update the output port = state
        discrete_state.get_mutable_vector().SetFromVector(tau)


######################################################################################################
#                     ########Define Trajectory Generator as a LeafSystem #######   
######################################################################################################
class JointSpaceValidityChecker(ob.StateValidityChecker):
    def __init__(self, si, check_fn, num_dof):
        super().__init__(si)
        self.check_fn = check_fn
        self.num_dof = num_dof

    def isValid(self, state):
        q = np.array([state[i] for i in range(self.num_dof)])
        return self.check_fn(q)

class MotionProfile(LeafSystem):
    def __init__(self, trajInit_):
        super().__init__()
        self.trajInit_ = trajInit_
        self.num_points = 40
        time_period = 3.0
        self.index = 0
        self.flag = True
        self.min_distance = 0.05
        update_period = time_period / self.num_points
        builder = DiagramBuilder()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.01)
        parser = Parser(plant)
        parser.AddModelsFromUrl("file://" + os.path.abspath(robot_path))
        base_link = plant.GetBodyByName("panda_link0")
        plant.WeldFrames(plant.world_frame(), base_link.body_frame())

        # --- A floating box ---
        # Add a model instance for the box
        box_model = plant.AddModelInstance("box")

        # Define mass and inertia for the box
        box_inertia = SpatialInertia(
            mass=0.5,                          # kg
            p_PScm_E=np.zeros(3),              # center of mass at the body origin
            G_SP_E=UnitInertia(0.01, 0.01, 0.01)  # simple rotational inertia
        )
        box = plant.AddRigidBody("Box", box_model, box_inertia)

        # Define box shape (10cm cube)
        box_shape = DrakeBox(0.4, 0.05, 0.4)

        # Add collision geometry to the box
        plant.RegisterCollisionGeometry(
            box,
            RigidTransform(),   # Pose relative to the box frame
            box_shape,
            "box_collision",
            CoulombFriction(0.9, 0.8)
        )

        # Add visual geometry to the box
        plant.RegisterVisualGeometry(
            box,
            RigidTransform(),
            box_shape,
            "box_visual",
            [0.8, 0.2, 0.2, 1.0]  # red color
        )
        box_pose = RigidTransform([0.6, 0.0, 0.3])
        plant.WeldFrames(plant.world_frame(), box.body_frame(), box_pose)
        plant.Finalize()
        diagram = builder.Build()
        diagram_context = diagram.CreateDefaultContext()
        self.plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)
        self.plant = plant
        self.diagram = diagram
        self.box = plant.GetModelInstanceByName("box")
        self.robot = plant.GetModelInstanceByName("panda")
        self.num_positions = self.plant.num_positions(self.robot)
        self._q_desired_port = self.DeclareVectorInputPort(name="q_desired", size=self.num_positions)
        self._state_port = self.DeclareVectorInputPort(name="state", size=2*self.num_positions)

        q_ctrl = self.DeclareDiscreteState(self.num_positions)
        self.DeclareStateOutputPort("q_ctrl", q_ctrl)
        
        self.DeclarePeriodicDiscreteUpdateEvent(
            period_sec= update_period,
            offset_sec=0.0,
            update=self.compute_trajectory
        )

    def compute_trajectory(self, context, discrete_state):
        """
        Periodically updates the next target joint configuration (q_next)
        along a collision-free path planned by RRTConnect.

        This function is triggered by a discrete update event inside Drake’s
        simulation loop. It computes a sequence of joint-space waypoints
        that guide the robot from its current configuration to the desired
        goal configuration while avoiding obstacles.

        Parameters
        ----------
        context : pydrake.systems.framework.Context
            Provides access to the current simulation time and inputs.
        discrete_state : pydrake.systems.framework.DiscreteValues
            The system's internal discrete state vector (used to store q_next).
        """

        # q_desired: the desired goal configuration provided by an external node
        q_desired = self._q_desired_port.Eval(context)
        # state: the current robot joint positions + velocities
        state = self._state_port.Eval(context)

        # Extract current joint positions
        q_i = state[:self.num_positions]
        # Update the plant’s internal configuration for accurate collision checking
        self.plant.SetPositions(self.plant_context, q_i)

        if self.flag:
            # Run OMPL’s RRT-Connect planner to compute a joint-space path
            self.path = self.plan_joint_space(q_i, q_desired, timeout=4)

            # If planner fails, fall back to holding the current position
            if self.path is None:
                self.path = q_i[np.newaxis, :]
            
            self.index = 0
            self.flag = False

        # If a valid path exists, follow it step by step 
        if self.path is not None and len(self.path) > self.index:
            # Select the next waypoint on the planned path
            q_next = self.path[self.index]

            # Advance to the next waypoint if we are close enough to the current one
            if np.linalg.norm(q_next - state[:self.num_positions]) < 0.2:
                self.index += 1
        else:
            # End of path reached -> hold final configuration
            q_next = self.path[-1]

        # Update the system’s discrete output (next target configuration)
        discrete_state.get_mutable_vector().SetFromVector(q_next)

        
    def check_configuration_validity(self, q):

        # This updates the positions of all robot bodies in the scene.
        self.plant.SetPositions(self.plant_context, q)

        # This provides access to Drake's geometry engine for computing
        # distances and collision information at the current configuration.
        query_object = self.plant.get_geometry_query_input_port().Eval(self.plant_context)

        # Retrieve an inspector to map geometry IDs to frames/bodies.
        inspector = query_object.inspector()

        # Compute signed distances between all geometry pairs.
        distances = query_object.ComputeSignedDistancePairwiseClosestPoints()

        # Iterate and find the smallest *relevant* distance
        min_dist = float("inf")

        for pair in distances:

            # Extract the two bodies associated with this distance pair.
            body_A, body_B = self._get_bodies_from_pair(pair, inspector)

            # Retrieve the model names (e.g., "panda", "box") for filtering.
            robot_A_name = self._get_robot_name(body_A)
            robot_B_name = self._get_robot_name(body_B)

            # Skip if both belong to the same body or same model instance
            if robot_A_name == robot_B_name:
                continue

            # Otherwise, consider this pair
            min_dist = min(min_dist, pair.distance)

        # If nothing relevant found, it's valid
        if min_dist == float("inf"):
            return True

        # Check against threshold
        return min_dist >= self.min_distance
        
    def plan_joint_space(self, q_start, q_goal, timeout=3):
        """
        Plans a collision-free path in the robot's joint space using OMPL's RRT-Connect algorithm.

        Parameters
        ----------
        q_start : array-like
            The starting joint configuration of the robot.
        q_goal : array-like
            The desired target joint configuration.
        timeout : float
            The maximum time allowed for the planner to search for a valid path (in seconds).

        Returns
        -------
        sampled : np.ndarray or None
            A set of intermediate joint configurations representing the collision-free path.
            Returns None if the planner fails to find a path within the timeout.
        """
        num_dof = self.num_positions                         # number of joints in the manipulator
        space = ob.RealVectorStateSpace(num_dof)             # N-dimensional Euclidean space ℝⁿ
        
        # Set the upper and lower limits for each joint
        bounds = ob.RealVectorBounds(num_dof)
        lower_limits = self.plant.GetPositionLowerLimits()
        upper_limits = self.plant.GetPositionUpperLimits()

        # Apply per-joint bounds to the OMPL state space
        for i in range(num_dof):
            bounds.setLow(i, lower_limits[i])
            bounds.setHigh(i, upper_limits[i])
        space.setBounds(bounds)

        # Configure the SpaceInformation (contains state validity)
        si = ob.SpaceInformation(space)
        validity_checker = JointSpaceValidityChecker(si, self.check_configuration_validity, num_dof)
        si.setStateValidityChecker(validity_checker) # geometry engine to ensure the robot is collision-free.
        si.setup()

         # Define start and goal configurations
        start = ob.State(space)
        goal = ob.State(space)
        for i in range(num_dof):
            start[i] = q_start[i]
            goal[i] = q_goal[i]

        # Problem definition: connect q_start -> q_goal with a feasible path
        pdef = ob.ProblemDefinition(si)
        # The last argument (1e-2) is the acceptable goal tolerance in joint space
        pdef.setStartAndGoalStates(start, goal, 1e-2)

        # Choose the planner (RRT-Connect) and initialize it
        planner = og.RRTConnect(si)             # create planner object
        planner.setProblemDefinition(pdef)      # assign start/goal/problem
        planner.setup()                         # initialize internal structures
        
        # The planner will attempt to find a collision-free path
        # connecting q_start and q_goal within the given timeout.
        solved = planner.solve(timeout)
        # Extract, simplify, and interpolate the path if found
        if solved:
            path = pdef.getSolutionPath()

            # Simplify the path by shortcutting redundant waypoints
            simplifier = og.PathSimplifier(si)
            simplifier.ropeShortcutPath(path)

            # Interpolate to obtain evenly spaced samples along the path
            path.interpolate(self.num_points)

            # Convert OMPL path states into a NumPy array [N × num_dof]
            sampled = np.array([
                            [state[i] for i in range(num_dof)]
                            for index, state in enumerate(path.getStates())
                        ])
            return sampled
        else:
            print("RRT failed to find a path.")
            return None
    
    def _get_bodies_from_pair(self, pair, inspector):
        # Get the frame IDs for the two geometries in the distance pair.
        frame_id_A = inspector.GetFrameId(pair.id_A)
        frame_id_B = inspector.GetFrameId(pair.id_B)

        # Map the frame IDs to their corresponding Body objects in the plant.
        body_A = self.plant.GetBodyFromFrameId(frame_id_A)
        body_B = self.plant.GetBodyFromFrameId(frame_id_B)
        
        return body_A, body_B
    
    def _get_robot_name(self, body):
        # Retrieve the human-readable name of that model instance.
        # This allows you to distinguish which robot or object the body comes from.

        model_instance = body.model_instance()
        return self.plant.GetModelInstanceName(model_instance)
######################################################################################################

# Function to Create Simulation Scene
def create_sim_scene(sim_time_step):   
    builder = DiagramBuilder()
    trajectory_mode = "trapezoidal"  # Options: "trapezoidal" or "s_curve"
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=sim_time_step)
    parser = Parser(plant)
    parser.AddModelsFromUrl("file://" + os.path.abspath(robot_path))
    base_link = plant.GetBodyByName("panda_link0")  # replace with your robot’s root link name
    plant.WeldFrames(plant.world_frame(), base_link.body_frame())

    # --- A floating box ---
    # Add a model instance for the box
    box_model = plant.AddModelInstance("box")

    # Define mass and inertia for the box
    box_inertia = SpatialInertia(
        mass=0.5,                          # kg
        p_PScm_E=np.zeros(3),              # center of mass at the body origin
        G_SP_E=UnitInertia(0.01, 0.01, 0.01)  # simple rotational inertia
    )
    box = plant.AddRigidBody("Box", box_model, box_inertia)

    # Define box shape (10cm cube)
    box_shape = DrakeBox(0.4, 0.01, 0.4)

    # Add collision geometry to the box
    plant.RegisterCollisionGeometry(
        box,
        RigidTransform(),   # Pose relative to the box frame
        box_shape,
        "box_collision",
        CoulombFriction(0.9, 0.8)
    )

    # Add visual geometry to the box
    plant.RegisterVisualGeometry(
        box,
        RigidTransform(),
        box_shape,
        "box_visual",
        [0.8, 0.2, 0.2, 1.0]  # red color
    )
    box_pose = RigidTransform([0.5, 0.0, 0.2])
    plant.WeldFrames(plant.world_frame(), box.body_frame(), box_pose)
    plant.Finalize()
    box = plant.GetModelInstanceByName("box")
    robot = plant.GetModelInstanceByName("panda")
    # Set the initial joint position of the robot otherwise it will correspond to zero positions
    q_start = [-1.0, -0, 0.0, -2.356, 0.0, 1.571, 0.785, 0.0, 0.0]
    plant.SetDefaultPositions(robot, q_start)
    print(plant.GetDefaultPositions())


    # Add visualization to see the geometries in MeshCat
    AddDefaultVisualization(builder=builder, meshcat=meshcat)

    # Add a PD+G controller to regulate the robot
    controller = builder.AddNamedSystem("PD+G controller", Controller(plant, robot))
    
    # Create a constant source for desired positions
    q_target = [ 2.5, -0.31, -2.03, -2.36, -0.44, 2.46, 2.67, 0.0, 0.0]
    path_planner = builder.AddNamedSystem(
        "Motion Profile",
        MotionProfile(trajInit_=q_start)
    )
    des_pos = builder.AddNamedSystem("Desired position", ConstantVectorSource(q_target))
    
    # Connect systems: plant outputs to controller inputs, and vice versa
    builder.Connect(plant.GetOutputPort("panda_state"), controller.GetInputPort("Current_state")) 
    builder.Connect(plant.GetOutputPort("panda_state"), path_planner.GetInputPort("state")) 
    builder.Connect(controller.GetOutputPort("tau_u"), plant.GetInputPort("panda_actuation"))
    builder.Connect(des_pos.get_output_port(), path_planner.GetInputPort("q_desired"))
    builder.Connect(path_planner.get_output_port(0), controller.GetInputPort("Desired_state"))


    logger_state = LogVectorOutput(plant.GetOutputPort("panda_state"), builder)
    logger_state.set_name("State logger")

    logger_traj = LogVectorOutput(path_planner.get_output_port(0), builder)
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

    maybe_save_block_diagram(diagram, "figures/block_diagram_04_path_planner.png")
    
    # Run simulation and record for replays in MeshCat
    meshcat.StartRecording()
    simulator.AdvanceTo(10.0)  # Adjust this time as needed
    meshcat.PublishRecording()

    # At the end of the simulation
    plot_joint_tracking(logger_state, logger_traj, simulator.get_context())

# Run the simulation with a specific time step. Try gradually increasing it!
run_simulation(sim_time_step=0.01)
