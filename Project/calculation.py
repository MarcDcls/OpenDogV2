import numpy as np
import pinocchio as pin

from open_dog import OpenDog
from trajectory import *

##################################### Pinocchio calculation ###########################################
# opendog = OpenDog()

def calculMotors_trajectoryUpAndDown(robot):
    traj = trajectoryUpAndDown(robot)#List of cartesians positions

    pin.forwardKinematics(robot.model, robot.data, robot.motors)
    pin.updateFramePlacements(robot.model, robot.data)

    # initialization of list of motor configuration : list of np.array
    res = [robot.motors]

    for j in range(len(traj)):
        robot.updateMotors(res[-1])
        pin.forwardKinematics(robot.model, robot.data, res[-1])
        pin.updateFramePlacements(robot.model, robot.data)
        pin.computeJointJacobians(robot.model, robot.data, res[-1])

        #Get real cartesian position
        X = np.array([])
        for i in robot.pinFrames:
            X = np.hstack((X, robot.getFrameXYZRPY(i)))

        #Error calculation : dX
        dX = np.subtract(traj[j], X)

        #Jacobian calculation -> J = [ J_CF, J_FL, J_FR, J_BL, J_BR ]
        for k in robot.pinFrames:
            # oRf_tmp = opendog.data.oMf[k].rotation
            # fJ_tmp = pin.computeFrameJacobian(opendog.model, opendog.data, res[-1] , k)[2]
            # print(fJ_tmp)
            # J_tmp=oRf_tmp@fJ_tmp
            if k == 3:
                J = pin.computeFrameJacobian(robot.model, robot.data, res[-1], k, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
                # J = pin.computeFrameJacobian(robot.model, robot.data, res[-1], k, pin.ReferenceFrame.WORLD)
                # J = pin.computeFrameJacobian(robot.model, robot.data, res[-1], k, pin.ReferenceFrame.LOCAL)
                #print("new J",J)
            else:
                J_tmp = pin.computeFrameJacobian(robot.model, robot.data, res[-1], k, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
                # J_tmp = pin.computeFrameJacobian(robot.model, robot.data, res[-1], k, pin.ReferenceFrame.WORLD)
                # J_tmp = pin.computeFrameJacobian(robot.model, robot.data, res[-1], k, pin.ReferenceFrame.LOCAL)
                J = np.vstack((J, J_tmp))
                #print("new J",J_tmp)

        #Angular position variation
        dq = J.T @ dX

        #New angular position
        q = np.add(res[-1], dq)

        res.append(q)

    return res












