import time

import pybullet as p
import pybullet_data

from calculation import calculMotors_trajectoryUpAndDown
from calculation_tools_simu import *
from action_tools_simu import *
from trajectory import trajectoryUpAndDown
from open_dog import OpenDog

simu = True

################################ SIMULATION ################################

TICK_RATE = 1 / 240 # Frequence a laquelle le simulateur s'actualise

if simu :
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    planeId = p.loadURDF("plane.urdf")
    p.setGravity(0, 0, -10)

## CREATE ROBOT ##
opendog = OpenDog(simu)
# pin.forwardKinematics(opendog.model, opendog.data, opendog.motors)
# pin.updateFramePlacements(opendog.model, opendog.data)

## SET TRAJECTORY ##
traj = trajectoryUpAndDown(opendog, dist=0.08)

# Initialization time
for h in range(int(3 * TICK_RATE)):
    p.stepSimulation()  
    time.sleep(TICK_RATE)

# Simulation loop
for i in range(100000):
    p.stepSimulation()

    q=[math.pi/4, math.pi/4, math.pi/4, math.pi/4, math.pi/4, math.pi/4, math.pi/4, math.pi/4, math.pi/4, math.pi/4, math.pi/4, math.pi/4]
    # if i < len(traj):
    #     opendog.updateMotors()
    #     pin.forwardKinematics(opendog.model, opendog.data, opendog.motors)
    #     pin.updateFramePlacements(opendog.model, opendog.data)
    #     pin.computeJointJacobians(opendog.model, opendog.data, opendog.motors)
    #
    #     # Get real cartesian position
    #     X = np.array([])
    #     for frame in opendog.URDFframes:
    #         X = np.hstack((X, getFrameCartesianPosition(opendog, frame)))
    #     dX = np.subtract(traj[i-100], X)
    #
    #     # Jacobian calculation -> J = [ J_CF | J_FL | J_FR | J_BL | J_BR ]
    #     for k in opendog.pinFrames:
    #         if k == 3:
    #             # J = pin.getFrameJacobian(opendog.model, opendog.data, k, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    #             J = pin.getFrameJacobian(opendog.model, opendog.data, k, pin.ReferenceFrame.LOCAL)
    #             # J = pin.getFrameJacobian(opendog.model, opendog.data, k, pin.ReferenceFrame.WORLD)
    #             # J = pin.computeFrameJacobian(opendog.model, opendog.data, res[-1], k, pin.ReferenceFrame.WORLD)
    #             # print("new J",J)
    #         else:
    #             # J_tmp = pin.getFrameJacobian(opendog.model, opendog.data, k, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    #             J_tmp = pin.getFrameJacobian(opendog.model, opendog.data, k, pin.ReferenceFrame.LOCAL)
    #             # J_tmp = pin.getFrameJacobian(opendog.model, opendog.data, k, pin.ReferenceFrame.WORLD)
    #             # J_tmp = pin.computeFrameJacobian(opendog.model, opendog.data, res[-1], k, pin.ReferenceFrame.WORLD)
    #             J = np.vstack((J, J_tmp))
    #             # print("new J",J_tmp)
    #
    #     # Angular position variation
    #     dq = J.T @ dX
    #
    #     # New angular position
    #     q = np.add(opendog.motors, dq)
    if i == 20 :
        moveMotors(opendog, q)
        getAngularPositionAllLegs(opendog)

    time.sleep(TICK_RATE)

p.disconnect()
