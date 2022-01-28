#https://github.com/stack-of-tasks/pinocchio/blob/master/examples/inverse-kinematics.py

# a tester dans simulation

from __future__ import print_function

import numpy as np
from numpy.linalg import norm, solve

import pinocchio
from pinocchio.visualize import MeshcatVisualizer

model, collision_model, visual_model = pinocchio.buildSampleModelManipulator()
data  = model.createData()

JOINT_ID = 6
oMdes = pinocchio.SE3(np.eye(3), np.array([1., 0., 1.]))

q      = pinocchio.neutral(model)
eps    = 1e-4
IT_MAX = 1000
DT     = 1e-1
damp   = 1e-12



viz = MeshcatVisualizer(model, collision_model, visual_model)

# Start a new MeshCat server and client.
# Note: the server can also be started separately using the "meshcat-server" command in a terminal:
# this enables the server to remain active after the current script ends.
#
# Option open=True pens the visualizer.
# Note: the visualizer can also be opened seperately by visiting the provided URL.
try:
    viz.initViewer(open=True)
except ImportError as err:
    print("Error while initializing the viewer. It seems you should install Python meshcat")
    print(err)
    sys.exit(0)

# Load the robot in the viewer.
viz.loadViewerModel()


i=0
while True:
    pinocchio.forwardKinematics(model,data,q)
    dMi = oMdes.actInv(data.oMi[JOINT_ID])
    err = pinocchio.log(dMi).vector
    if norm(err) < eps:
        success = True
        break
    if i >= IT_MAX:
        success = False
        break
    J = pinocchio.computeJointJacobian(model,data,q,JOINT_ID)
    v = - J.T.dot(solve(J.dot(J.T) + damp * np.eye(6), err))
    q = pinocchio.integrate(model,q,v*DT)
    if not i % 10:
        print('%d: error = %s' % (i, err.T))
    i += 1

if success:
    print("Convergence achieved!")
    viz.display(q)
    viz.displayCollisions(False)
    viz.displayVisuals(False)
else:
    print("\nWarning: the iterative algorithm has not reached convergence to the desired precision")


print('\nresult: %s' % q.flatten().tolist())
print('\nfinal error: %s' % err.T)