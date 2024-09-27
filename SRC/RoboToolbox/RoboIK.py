import roboticstoolbox as rtb
import matplotlib.pyplot as plt
import numpy as np
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
        self.Qbest = []
    
# Create 3D grid of Positions    
# region   
Xspan = np.linspace(-1.5,1.5,11)
Yspan = np.linspace(-1.5,1.5,11)
Zspan = np.linspace(0,1.5,6)
WorkspaceGrid = np.array(np.meshgrid(Xspan, Yspan, Zspan)).T.reshape(-1, 3)
WorkspaceNodes = []
# endregion


# Sample Solve
Solver = rtb.IK_LM()
Tep = pandarobo.fkine(Qrest)
solution = pandarobo.ik_LM(Tep)
#solution = pandarobo.ik_LM(Tep,[qstart, iterliimit, seachlimit, tolerances, tolerancemask, other])


# IK Solve
# region
print ("Start IK Solve Loop")
startT = time.time()

for X in WorkspaceGrid:
    print(X)
    Tep = [[1,0,0,X[0]] , [0,1,0,X[1]] , [0,0,1,X[2]] , [0,0,0,1]]
    solution = pandarobo.ik_LM(Tep)
    
    # Solutions
    # [0] = Solution Config
    # [1] = Boolean Success Variable
    # [2] = iterations count
    # [3] = searches
    # [4] = residual error from cost function
    
    if solution[1] == 1:
        print("SolutionFound")
    else:
        print("No Sol")

    
# endregion



plt.ioff()
print("End")