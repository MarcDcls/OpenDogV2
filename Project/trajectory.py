import numpy as np
import operator
import pinocchio as pin
import pybullet as p


def trajectoryUpAndDown(robot, length=500, dist=0.2, orientation=True):
    """
    Generate a trajectory of cartesian coordinates of an up and down movement

    :param robot: Robot object
    :param length: number of positions
    :param dist: height of the step / distance on z
    :param orientation: if true generate xyzrpy coordinates, else generate xyz coordinates
    :return: List of cartesian coordinates
    """
    pin.forwardKinematics(robot.model, robot.data, robot.motors)
    pin.framesForwardKinematics(robot.model, robot.data, robot.motors)

    X0 = np.array([])
    if orientation:
        vect = [0, 0, ((2 * dist) / length), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0]
        if robot.simu:
            for i in robot.URDFframes:
                X0 = np.hstack((X0, p.getLinkState(robot.id, i)[:2][0]))
                X0 = np.hstack((X0, p.getLinkState(robot.id, i)[:2][1][:-1]))
        else:
            for i in robot.pinFrames:
                X0 = np.hstack((X0, robot.getFrameXYZRPY(i)))
    else:
        vect = [0, 0, ((2 * dist) / length), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        if robot.simu:
            for i in robot.URDFframes:
                X0 = np.hstack((X0, p.getLinkState(robot.id, i)[0]))
        else:
            for frame in robot.pinFrames:
                X0 = np.hstack((X0, robot.data.oMf[frame].translation))

    traj = [X0]
    for i in range(length // 2):
        next_X = list(map(operator.sub, traj[-1], vect))
        traj.append(next_X)
    for j in range(length // 2):
        next_X = list(map(operator.add, traj[-1], vect))
        traj.append(next_X)

    return traj

def footTrajectory(robot, idFrame, t, orientation=True):
    """
    Generate a trajectory of cartesian coordinates for an given foot moving in the x-axis
    inspired by : https://github.com/EtienneAr/ISAE-PIE-quadruped-solo_pybullet/blob/master/solo_pybullet/controller.py

    :param robot:  Robot object
    :param idFrame: Frame identifiant
    :param t:
    :param orientation: if true generate xyzrpy coordinates, else generate xyz coordinates
    :return: List of cartesian coordinates
    """
    traj = []
    T = 0.2  # period of the foot trajectory
    dx = 0.03  # displacement amplitude by x
    dz = 0.06  # displacement amplitude by z
    x = []
    y = []
    z = []
    if orientation:
        return traj
    else:
        xyz = robot.data.oMf[idFrame].translation
        x0 = xyz[0]
        y0 = xyz[1]
        z0 = xyz[2]
        if t >= T:
            t %= T
        x.append(x0 - dx * np.cos(2 * np.pi * t / T))
        if t <= T / 2.:
            z.append(z0 + dz * np.sin(2 * np.pi * t / T))
        else:
            z.append(0)
        return traj
