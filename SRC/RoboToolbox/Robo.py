import roboticstoolbox as rtb
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import time
plt.ion()

# Load Panda Robot as pandarobo
# region
pandarobo = rtb.models.DH.Panda()
Qrest = pandarobo.qr
Jacobi = pandarobo.jacob0(Qrest)
GravTorque = pandarobo.gravload(Qrest)

MinPosArr = pandarobo.qlim[0]
MaxPosArr = pandarobo.qlim[1]
JointLims = np.stack((MinPosArr, MaxPosArr))
JointLims = np.transpose(JointLims)
# endregion

# Create Joint Grid Positon Array
# region
J0Range = np.linspace( JointLims[0][0], JointLims[0][1], 5)
J1Range = np.linspace( JointLims[1][0], JointLims[1][1], 5)
J2Range = np.linspace( JointLims[2][0], JointLims[2][1], 5)
J3Range = np.linspace( JointLims[3][0], JointLims[3][1], 5)
J4Range = np.linspace( JointLims[4][0], JointLims[4][1], 5)
J5Range = np.linspace( JointLims[5][0], JointLims[5][1], 5)
J6Range = np.linspace( JointLims[6][0], JointLims[6][1], 5)
Pos = np.array(np.meshgrid(J0Range,J1Range,J2Range,J3Range,J4Range,J5Range,J6Range)).T.reshape(-1, 7)
# endregion

# Create set of position bins in 3D space (Each)
# region
class ThreeDBin:
    def __init__ (self,Xrange,Yrange,Zrange):
        self.Xrange = Xrange
        self.Yrange = Yrange
        self.Zrange = Zrange
        self.Qpos = []
        self.MaxManip = 0
        self.Qbest = []
    
    def inBin(self, EndEffector, QJoints):
        if((EndEffector[0] >= self.Xrange[0] and EndEffector[0] < self.Xrange[1]) and 
        (EndEffector[1] >= self.Yrange[0] and EndEffector[1] < self.Yrange[1]) and
        (EndEffector[2] >= self.Zrange[0] and EndEffector[2] < self.Zrange[1])):
            self.Qpos.append(QJoints)
            return True
        else:
            return False
       
Xspan = np.linspace(-1,1,6)
Yspan = np.linspace(-1,1,6)
Zspan = np.linspace(0,1,11)

WorkspaceBins = []
for i in range(len(Xspan)-1):
    for j in range(len(Yspan)-1):
        for k in range(len(Zspan)-1):
            # print("XRange" , Xspan[i] , Xspan[i+1],
            #       "YRange" , Yspan[j] , Yspan[j+1],
            #       "ZRange" , Zspan[k] , Zspan[k+1],)
            Temp = []
            XRange = [Xspan[i], Xspan[i+1]]
            YRange = [Yspan[j], Yspan[j+1]]
            ZRange = [Zspan[k], Zspan[k+1]]
            Temp = ThreeDBin(XRange, YRange,ZRange)
            WorkspaceBins.append(Temp)
            
# Loop Through Joint Position Combinations, bin into workspace bins based on end effector location

print ("Start Eval Loop")
startT = time.time()
for X in Pos:
    EndEffectorPos = pandarobo.fkine(X)
    EffectorPos = [EndEffectorPos.A[0][3], EndEffectorPos.A[1][3], EndEffectorPos.A[2][3]]
    for Bins in WorkspaceBins:
        if Bins.inBin(EffectorPos,X):
            break
finishT = time.time()
print("Time Elapsed = ", finishT-startT)


# Plot the robot in a specific configuration
env = pandarobo.plot(pandarobo.qz, block=False)  # env is the environment object
# Extract the 3D axes from the environment
ax = env.ax  # Access the Axes3D object from the environment

def add_cuboid(ax, origin, size, alphaval):
    o = np.array(origin)
    l = size[0]
    w = size[1]
    h = size[2]
    vertices = np.array([[0, 0, 0], [l, 0, 0], [l, w, 0], [0, w, 0],  # bottom face
                         [0, 0, h], [l, 0, h], [l, w, h], [0, w, h]]) + o

    faces = [[vertices[j] for j in [0, 1, 2, 3]],  # bottom face
             [vertices[j] for j in [4, 5, 6, 7]],  # top face
             [vertices[j] for j in [0, 1, 5, 4]],  # side face
             [vertices[j] for j in [2, 3, 7, 6]],  # opposite side face
             [vertices[j] for j in [0, 3, 7, 4]],  # front face
             [vertices[j] for j in [1, 2, 6, 5]]]  # back face

    # Add the cuboid to the plot with full opacity
    ax.add_collection3d(Poly3DCollection(faces, facecolors='red', linewidths=1, edgecolors='r', alpha=.1 * alphaval))

JointTLims = np.array([100,100,100,100,100,100,100])
UnitWrench = [0, 0, -1, 0, 0,0]
HighestManipGlobal = 0
print("Begin Filtering Bins")
startT = time.time()
for bins in WorkspaceBins:
    if not bins.Qpos:
        continue
    else:
        MaxManipBin = bins.MaxManip
        QCandidate = Qrest
        # Find Max Manipulability in Bin
        for Config in bins.Qpos:
            # ForceCalcs
            Manip = pandarobo.manipulability(Config)
            if MaxManipBin < Manip:
                MaxManipBin = Manip
                QCandidate = Config
                bins.MaxManip = Manip
                bins.Qbest = Config
        HighestManipGlobal = max(HighestManipGlobal,MaxManipBin)
finishT = time.time()
print("Time Elapsed = ", finishT-startT)

# Iterate Through Bins, plot cuboids
print("Begin Plotting")
startT = time.time()
for bins in WorkspaceBins:
    if not bins.Qpos:
        continue
    else:
        origin = [bins.Xrange[0] ,bins.Yrange[0], bins.Zrange[0]]
        size = [bins.Xrange[1]-bins.Xrange[0], bins.Yrange[1]-bins.Yrange[0], bins.Zrange[1]-bins.Zrange[0]]
        alpha = 1 - ((HighestManipGlobal - bins.MaxManip)/HighestManipGlobal)
        add_cuboid(ax, origin, size, alpha)
finishT = time.time()
print("Time Elapsed = ", finishT-startT)

# Force a refresh of the plot
plt.draw()
plt.pause(1)  # Ensure the figure refreshes

# Keep the plot open for interaction
plt.show()
print("End")