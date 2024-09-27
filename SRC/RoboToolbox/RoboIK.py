import roboticstoolbox as rtb
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import ConvexHull
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib import cm
from matplotlib.colors import Normalize
import time
plt.ion()

# Load Panda Robot as pandarobo
# region
pandarobo = rtb.models.DH.Panda()
Qrest = pandarobo.qr

MinPosArr = pandarobo.qlim[0]
MaxPosArr = pandarobo.qlim[1]
JointLims = np.stack((MinPosArr, MaxPosArr))
JointLims = np.transpose(JointLims)
# endregion

# Class/Structure corresponding to a given workspace bin
class PointSpace:
    def __init__ (self, Point):
        self.Position = [Point[0], Point[1], Point[2]]
        self.ConfigList = []
        self.MaxManip = 0
        self.BestSol = []
    
# Create 3D grid of Positions    
# region   
# Dense Span
Xspan = np.linspace(-1.5,1.5,31)
Yspan = np.linspace(-1.5,1.5,31)
Zspan = np.linspace(0,1.5,16)

# Sparse Span
# Xspan = np.linspace(-1.5,1.5,11)
# Yspan = np.linspace(-1.5,1.5,11)
# Zspan = np.linspace(0,1.5,6)

WorkspaceGrid = np.array(np.meshgrid(Xspan, Yspan, Zspan)).T.reshape(-1, 3)
WorkspaceNodes = []
# endregion

# IK Solve
# region
print ("Start IK Solve Loop")
startT = time.time()
NodeField = []
HighManip = 0
for X in WorkspaceGrid:
    Tep = [[1,0,0,X[0]] , [0,1,0,X[1]] , [0,0,1,X[2]] , [0,0,0,1]]
    solution = pandarobo.ik_LM(Tep,joint_limits = True)
    
    # Solutions
    # [0] = Solution Config
    # [1] = Boolean Success Variable
    # [2] = iterations count
    # [3] = searches
    # [4] = residual error from cost function
    
    if solution[1] == 1:
        NewNode = []                # Make Empty
        NewNode = PointSpace(X)     # Create Temp Node
        NewNode.ConfigList.append(solution[0])
        Manip = pandarobo.manipulability(solution[0])
        HighManip = max(HighManip,Manip)
        NewNode.MaxManip = Manip
        NewNode.BestSol = solution
        WorkspaceNodes.append(NewNode)
        NodeField.append(X)

        
finishT = time.time()
print("Time Elapsed = ", finishT-startT)
# endregion

# Plot Workspace Bubble
# region
ConvHull = ConvexHull(NodeField)
# Create a new figure for 3D plotting

env = pandarobo.plot(pandarobo.qr, block=False)  # env is the environment object
ax = env.ax  # Access the Axes3D object from the environment

# Plot the original points
points = np.array(NodeField)
# ax.scatter(points[:, 0], points[:, 1], points[:, 2], color='black', marker='o', s=50, label='Points')

for simplex in ConvHull.simplices:
    # Extract the vertices of the simplex
    triangle = points[simplex]
    # Create a Poly3DCollection for the triangle
    tri = Poly3DCollection([triangle], alpha=0.15)
    tri.set_color('cyan')
    tri.set_edgecolor('black')
    ax.add_collection3d(tri)

ax.set_xlim([-1.5, 1.5])
ax.set_ylim([-1.5, 1.5])
ax.set_zlim([0, 1.5])

# Force a refresh of the plot
ax.view_init(30,-60)
plt.draw()
plt.pause(1)  # Ensure the figure refreshes
plt.show()
filename = f'SRC\RoboToolbox\RoboData\Data_WorkspaceBubbleV1.png'
plt.savefig(filename)

# Force a refresh of the plot
ax.view_init(30,60)
plt.draw()
plt.pause(1)  # Ensure the figure refreshes
plt.show()
filename = f'SRC\RoboToolbox\RoboData\Data_WorkspaceBubbleV2.png'
plt.savefig(filename)
plt.close()
# endregion



# Create Scatter Plot for Each Accessible Node

vmin = 0
vmax = HighManip
norm = Normalize(vmin=vmin, vmax = vmax)
colormap = cm.get_cmap('turbo')

for Zpoint in Zspan:
    env = pandarobo.plot(pandarobo.qr, block=False)  # env is the environment object
    ax = env.ax  # Access the Axes3D object from the environment

    ZNodeField = []
    ZNodeColor = []
    for Node in WorkspaceNodes:
        if Node.Position[2] == Zpoint:
            ZNodeField.append(Node.Position)
            ZNodeColor.append(Node.MaxManip)
    
    ZNodeField = np.array(ZNodeField)
    ZNodeColor = np.array(ZNodeColor)
    if ZNodeField.any():
        ax.scatter(ZNodeField[:, 0], ZNodeField[:, 1], ZNodeField[:, 2], color=colormap(norm(ZNodeColor)), marker='o', s=50, label='Points')
        
    ax.view_init(30,-60)
    ax.set_xlim([-1.5, 1.5])
    ax.set_ylim([-1.5, 1.5])
    ax.set_zlim([0, 1.5])

    plt.draw()
    plt.pause(1)  # Ensure the figure refreshes
    plt.show()
    filename = f'SRC\RoboToolbox\RoboData\IK_DataSet\IK' + str(round(Zpoint,2)) + 'slice.png'
    plt.savefig(filename)






print("End")