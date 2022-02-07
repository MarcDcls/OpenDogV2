import time

import pybullet as p
import pybullet_data

from calculation import calculMotors_trajectoryUpAndDown
from calculation_tools_simu import *
from action_tools_simu import *
from trajectory import trajectoryUpAndDown

## CREATE ROBOT ##
opendog = OpenDog()
pin.forwardKinematics(opendog.model, opendog.data, opendog.motors)
pin.updateFramePlacements(opendog.model, opendog.data)

## SET TRAJECTORY ##
traj = trajectoryUpAndDown(opendog)

################################ SIMULATION ################################

# FRAME_RATE = 1 / 240 # Frequence a laquelle un ordre est transmis au verins
TICK_RATE = 1 / 240 # Frequence a laquelle le simulateur s'actualise

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")
p.setGravity(0, 0, -10)

opendogSimu = p.loadURDF("urdf/robot.urdf", [0, 0, 0.6], p.getQuaternionFromEuler([0, 0, 0]))
motorsInitialization(opendogSimu)

# Initialization time
for h in range(int(3 * TICK_RATE)):
    p.stepSimulation()  
    time.sleep(TICK_RATE)

# initialization of list of motor configuration : list of np.array
res = [opendog.motors]

# Simulation loop
for i in range(10000):
    p.stepSimulation()
    print(i)

    if (i > 99) and (i < (100+len(traj))):
        opendog.updateMotors(res[-1])
        pin.forwardKinematics(opendog.model, opendog.data, res[-1])
        pin.updateFramePlacements(opendog.model, opendog.data)

        # Get real cartesian position
        X = np.array([])
        for frame in opendog.URDFframes:
            X = np.hstack((X, getFrameCartesianPosition(opendogSimu, frame)))

        # Error calculation : dX
        dX = np.subtract(traj[i-100], X)

        # Jacobian calculation -> J = [ J_CF, J_FL, J_FR, J_BL, J_BR ]
        for k in opendog.pinFrames:
            if k == 3:
                J = pin.getFrameJacobian(opendog.model, opendog.data, k, pin.ReferenceFrame.WORLD)
                # J = pin.computeFrameJacobian(opendog.model, opendog.data, res[-1], k, pin.ReferenceFrame.WORLD)
                # print("new J",J)
            else:
                J_tmp = pin.getFrameJacobian(opendog.model, opendog.data, k, pin.ReferenceFrame.WORLD)
                # J_tmp = pin.computeFrameJacobian(opendog.model, opendog.data, res[-1], k, pin.ReferenceFrame.WORLD)
                J = np.vstack((J, J_tmp))
                # print("new J",J_tmp)

        # Angular position variation
        dq = J.T @ dX

        # New angular position
        q = np.add(res[-1], dq)

        res.append(q)

        moveMotors(opendogSimu, res[-1])

    time.sleep(TICK_RATE)

p.disconnect()
