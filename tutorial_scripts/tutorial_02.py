"""
tutorial_02.py
---------------
Minimal Drake MultibodyPlant tutorial: Panda robot visualization and simulation.

This tutorial shows:
1. How to load a robot model (the Franka Emika Panda arm).
2. How to add extra bodies (a ground and a box) with collision and visual geometry.
3. How to inspect the contents of the plant (bodies and joints).
4. How to simulate the system and visualize it in MeshCat.
"""

import os
import shutil
import numpy as np
import pydot
from IPython.display import SVG, display

# Import necessary parts of Drake
from pydrake.geometry import StartMeshcat, SceneGraph, Box as DrakeBox
from pydrake.math import RigidTransform
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant, CoulombFriction
from pydrake.multibody.tree import SpatialInertia, UnitInertia
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.visualization import AddDefaultVisualization, ModelVisualizer


def maybe_save_block_diagram(diagram, image_path):
    """Save the diagram image when Graphviz is installed."""
    if shutil.which("dot") is None:
        print("Skipping block diagram export because Graphviz 'dot' was not found in PATH.")
        return

    svg_data = diagram.GetGraphvizString(max_depth=2)
    graph = pydot.graph_from_dot_data(svg_data)[0]
    graph.write_png(image_path)
    print(f"\nBlock diagram saved as: {image_path}")

# --------------------------------------------------------------------
# Global settings
# --------------------------------------------------------------------
# Start Meshcat visualizer: this will give you a link in the notebook/terminal
meshcat = StartMeshcat()

visualize = False  # If True: use ModelVisualizer interactively, if False: run a simulation
add_bodies = True  # If True: add ground and a box to the scene

# Path to Panda robot URDF
model_path = os.path.join(
    "..", "models", "descriptions", "robots", "arms", "franka_description", "urdf", "panda_arm_hand.urdf"
)

# --------------------------------------------------------------------
# Function: create_sim_scene
# --------------------------------------------------------------------
def create_sim_scene(sim_time_step):   
    """
    Create a simulation scene with the Panda robot and optional extra bodies.

    Args:
        sim_time_step (float): The discrete time step for the plant.

    Returns:
        diagram (Diagram): A system diagram with the robot, optional objects, and visualization.
    """
    # Clean up the Meshcat window (so we start from an empty scene each time)
    meshcat.Delete()
    meshcat.DeleteAddedControls()

    # A DiagramBuilder is where we construct our system
    builder = DiagramBuilder()

    # Add a MultibodyPlant (for physics) and a SceneGraph (for geometry/visualization)
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=sim_time_step)

    # Load the Panda robot from URDF
    panda_model = Parser(plant).AddModelsFromUrl("file://" + os.path.abspath(model_path))[0]

    # Fix the Panda base to the world so it doesn’t fall
    base_link = plant.GetBodyByName("panda_link0")
    plant.WeldFrames(plant.world_frame(), base_link.body_frame())

    # ----------------------------------------------------------------
    # Extra bodies (floor + box) if requested
    # ----------------------------------------------------------------
    if add_bodies:
        # --- Ground plane ---
        # Use a thin box as the floor so that it is visible in MeshCat.
        floor_shape = DrakeBox(4.0, 4.0, 0.02)
        X_WFloor = RigidTransform([0.0, 0.0, -0.01])
        plant.RegisterCollisionGeometry(
            plant.world_body(),
            X_WFloor,
            floor_shape,
            "ground_collision",
            CoulombFriction(0.9, 0.8)   # Friction coefficients
        )
        plant.RegisterVisualGeometry(
            plant.world_body(),
            X_WFloor,
            floor_shape,
            "ground_visual",
            [0.5, 0.5, 0.5, 1.0]  # RGBA color (gray)
        )

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
        box_shape = DrakeBox(0.1, 0.1, 0.1)

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

    # ----------------------------------------------------------------
    # Finalize plant: no more bodies or joints can be added after this
    # ----------------------------------------------------------------
    plant.Finalize()
  
    # ----------------------------------------------------------------
    # Inspect contents: print bodies and joints of the Panda
    # ----------------------------------------------------------------
    print("\nBodies in the Panda model:")
    for body_index in plant.GetBodyIndices(panda_model):
        print("  -", plant.get_body(body_index).name())

    print("\nJoints in the Panda model:")
    for joint_index in plant.GetJointIndices(panda_model):
        print("  -", plant.get_joint(joint_index).name())

    # ----------------------------------------------------------------
    # Set initial/default states
    # ----------------------------------------------------------------
    # Panda arm default joint configuration
    plant.SetDefaultPositions(panda_model, [
        0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.774, 0, 0
    ])
    
    if add_bodies:
        # ------------------------------------------------------------
        # Two ways of setting the initial pose of a floating body (the box)
        # ------------------------------------------------------------

        # (1) Using SetDefaultPositions:
        # This requires encoding the pose in the format
        # [qw, qx, qy, qz, x, y, z] for the quaternion + position.
        # Here: box is placed at (0.5, 0, 0.5) with unit quaternion [1, 0, 0, 0].
        plant.SetDefaultPositions(box_model, [1, 0, 0, 0, 0.5, 0.0, 0.5])  

        # (2) Using SetDefaultFloatingBaseBodyPose:
        # This is usually clearer because we work directly with a RigidTransform.
        # Here: we place the box at (0.5, 0, 0.5) with identity rotation.
        X_WBox = RigidTransform([0.5, 0.0, 0.5])
        plant.SetDefaultFloatingBaseBodyPose(box, X_WBox)

    # ----------------------------------------------------------------
    # Inspect initial state
    # ----------------------------------------------------------------
    plant_context = plant.CreateDefaultContext()
    print("\nInitial Panda joint state (positions + velocities):")
    print(plant.GetPositionsAndVelocities(plant_context, panda_model))
    
    if add_bodies:
        # For floating bodies we can inspect in two ways:
        # (a) From its model instance: quaternion + position + velocities
        print("\nInitial box state (pose + velocities):")
        print(plant.GetPositionsAndVelocities(plant_context, box_model))
        
        # (b) As a RigidTransform (rotation + translation matrix)
        print("\nInitial pose of the box (RigidTransform):")
        print(plant.GetFreeBodyPose(plant_context, box))


    # ----------------------------------------------------------------
    # Add visualization
    # ----------------------------------------------------------------
    AddDefaultVisualization(builder=builder, meshcat=meshcat)

    # Build the final diagram (plant + scene graph + viz)
    diagram = builder.Build()
    return diagram


# --------------------------------------------------------------------
# Function: run_simulation
# --------------------------------------------------------------------
def run_simulation(sim_time_step):
    """
    Either run an interactive visualizer, or simulate the system.
    """
    if visualize:
        # If visualize=True, just load and display the robot interactively
        visualizer = ModelVisualizer(meshcat=meshcat)
        visualizer.parser().AddModelsFromUrl("file://" + os.path.abspath(model_path))
        visualizer.Run()
        
    else:
        # Otherwise, build the scene and simulate
        diagram = create_sim_scene(sim_time_step)

        # Create and configure the simulator
        simulator = Simulator(diagram)
        simulator.set_target_realtime_rate(1.0)  # Try to match real time
        simulator.Initialize()

        sim_time = 5.0  # seconds of simulated time

        meshcat.StartRecording()         # Start recording the sim
        simulator.AdvanceTo(sim_time)    # Runs the simulation for sim_time seconds
        meshcat.PublishRecording()       # Publish recording to replay in Meshcat
            
        maybe_save_block_diagram(diagram, "figures/block_diagram_02.png")


# --------------------------------------------------------------------
# Run the simulation
# --------------------------------------------------------------------
# Try playing with the time step (e.g. 0.001 vs 0.01 vs 0.1)
run_simulation(sim_time_step=0.01)
