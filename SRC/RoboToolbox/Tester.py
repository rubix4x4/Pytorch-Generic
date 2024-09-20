import roboticstoolbox as rtb
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
plt.ion()
# Example: Plot a Puma 560 robot using the Peter Corke toolbox
robot = rtb.models.DH.Puma560()

# Plot the robot in a specific configuration
env = robot.plot(robot.qz, block=False)  # env is the environment object
# Extract the 3D axes from the environment
ax = env.ax  # Access the Axes3D object from the environment

# Function to add a 3D rectangle (cuboid) to the plot
def add_cuboid(ax, origin, size):
    o = np.array(origin)
    l, w, h = size
    vertices = np.array([[0, 0, 0], [l, 0, 0], [l, w, 0], [0, w, 0],  # bottom face
                         [0, 0, h], [l, 0, h], [l, w, h], [0, w, h]]) + o

    faces = [[vertices[j] for j in [0, 1, 2, 3]],  # bottom face
             [vertices[j] for j in [4, 5, 6, 7]],  # top face
             [vertices[j] for j in [0, 1, 5, 4]],  # side face
             [vertices[j] for j in [2, 3, 7, 6]],  # opposite side face
             [vertices[j] for j in [0, 3, 7, 4]],  # front face
             [vertices[j] for j in [1, 2, 6, 5]]]  # back face

    # Add the cuboid to the plot with full opacity
    ax.add_collection3d(Poly3DCollection(faces, facecolors='red', linewidths=1, edgecolors='r', alpha=1.0))

# Add a cuboid to the plot
add_cuboid(ax, origin=(0.5, 0.5, 0), size=(0.2, 0.2, 0.7))

# Adjust plot limits to make sure the cuboid is visible
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([0, 1])

# Adjust the viewpoint
ax.view_init(elev=30, azim=60)

# Force a refresh of the plot
plt.draw()
plt.pause(1)  # Ensure the figure refreshes

# Keep the plot open for interaction
plt.ioff()
plt.show()

print(f"Axes name: {ax.name}")
print(f"Axes limits: X={ax.get_xlim()}, Y={ax.get_ylim()}, Z={ax.get_zlim()}")

print("end")