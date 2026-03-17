# Robotics I
Tutorials and materials for the Robotics I practical sessions.

### Objectives
This course introduces the fundamental concepts used in robotics simulation and control. Through the tutorials in this repository, students will work with robot models in Drake, run simulations  and motion generation examples. By the end of the practical sessions, students should be able to:

1. Model robotic systems and simple environments.
2. Understand and simulate robot kinematics.
3. Generate and follow simple reference trajectories.

***Results demonstrated through:*** tutorial exercises, simulations, and course assignments.

## Practical scope
The practical sessions in this repository focus on:

- **System modeling**: building robot and environment models in Drake.

- **Simulation**: running and visualizing robot behavior in MeshCat.

- **Kinematics and Trajectory generation**: defining reference trajectories.

### How to start with the repo?

1. Follow the installation guide in [`00_Startup.md`](./tutorial_doc/00_Startup.md) for a complete setup of the repository and its dependencies.

2. Continue with the [tutorials](./tutorial_doc/), starting from [`01_Introduction.md`](./tutorial_doc/01_Introduction.md), for an overview of the simulation framework and its main functionalities.

3. Use the accompanying [Python scripts](./tutorial_scripts/) to run the examples from the tutorials. For example, the tutorial [`02_Modelling.md`](./tutorial_doc/02_Modelling.md) is accompanied by [`tutorial_02.py`](./tutorial_scripts/tutorial_02.py).

## Tutorial content
| File                                                                                                                | Summary                         |
| ---                                                                                                                 | ---                             |
| [00_Startup.md](./tutorial_doc/00_Startup.md)         | Installation guide for the project material and its dependencies, along with a guide to setting up Ubuntu on your machine. |
| [01_Introduction.md](./tutorial_doc/01_Introduction.md)         | Introduction to the simulation framework (Drake), its basic components, and what you can achieve with it. |
| [02_Modelling.md](./tutorial_doc/02_Modelling.md)                 | Modeling and simulating robots and environments in Drake using the `MultibodyPlant`. Includes visualization with MeshCat. |
| [03_Dynamics.md](./tutorial_doc/03_Dynamics.mdss)                 | Running simulations with dynamics. Introduction to Drake’s `LeafSystem` for designing system blocks, and building model-based controllers. |
| [04_TrajectoryPlanning.md](./tutorial_doc/04_TrajectoryPlanning.md)                 | 	Short introduction to trajectory planning: defining a reference trajectory, sending it to the controller, and visualizing results. |
---

# Troubleshooting & Support
If you find a bug in the repository, need assistance, or have any other questions, please open an issue in the repository **(recommended)** or contact one or more of the following by email:
* Mohayad Omer `Mohayad.Abdelmonim.Mahmoud.Omer@vub.be`
* Chaoyue Fei `Chaoyue.Fei@vub.be`
* Zemerart Asani `Zemerart.Asani@vub.be`

We will try to help you as soon as possible.
