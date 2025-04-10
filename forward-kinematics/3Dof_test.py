# 3dof_test.py
import numpy as np
import matplotlib.pyplot as plt
from kinematics import forward_kinematics, end_effector_position

# Define link lengths
L1, L2, L3 = 1.0, 0.5, 1.0

# Define joint angles
theta1 = np.radians(0)
theta2 = np.radians(-15)
theta3 = np.radians(45)

# Construct DH table
dh_table = [
    {'theta': theta1, 'd': 0, 'a': 0,   'alpha': 0},
    {'theta': theta2, 'd': 0, 'a': L1,  'alpha': -np.pi/2},
    {'theta': theta3, 'd': 0, 'a': L2,  'alpha': 0},
    {'theta': 0,      'd': 0, 'a': L3,  'alpha': 0},  
]

# Compute FK
Ts = forward_kinematics(dh_table, return_all=True)
pos = end_effector_position(Ts[-1])
print(f"End-effector position: x={pos[0]:.2f}, y={pos[1]:.2f}, z={pos[2]:.2f}")

# === Visualization ===
def draw_frames(ax, Ts, scale=0.5):
    for T in Ts:
        origin = T[:3, 3]
        x_axis = T[:3, 0] * scale
        y_axis = T[:3, 1] * scale
        z_axis = T[:3, 2] * scale

        ax.quiver(*origin, *x_axis, color='r')
        ax.quiver(*origin, *y_axis, color='g')
        ax.quiver(*origin, *z_axis, color='b')

def plot_arm(Ts):
    xs, ys, zs = [], [], []
    for T in Ts:
        p = T[:3, 3]
        xs.append(p[0])
        ys.append(p[1])
        zs.append(p[2])

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(xs, ys, zs, '-o', color='blue', linewidth=3, markersize=8)
    ax.set_box_aspect([1, 1, 1])  # Equal aspect ratio
    draw_frames(ax, Ts)
    ax.set_title("3-DOF Arm - FK Visualization")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_xlim([-2, 2])
    ax.set_ylim([-2, 2])
    ax.set_zlim([-2, 2])
    ax.view_init(elev=30, azim=45)
    ax.text(xs[-1], ys[-1], zs[-1], 'End-Effector', fontsize=10, color='purple')
    plt.tight_layout()
    plt.show()

plot_arm(Ts)
