import numpy as np
import operator
import pinocchio as pin
import pybullet as p

from open_dog import OpenDog

def trajectoryUpAndDown(opendog, length=500, dist=0.2, orientation=True):
    """
    Generate a trajectory of cartesian coordinates

    :param opendog: Robot object
    :param length: number of positions
    :param dist: heigth of the step / distance on z
    :param orientation:
    :return: List of cartesian coordinates
    """
    pin.forwardKinematics(opendog.model, opendog.data, opendog.motors)
    pin.framesForwardKinematics(opendog.model, opendog.data, opendog.motors)

    X0 = np.array([])# vector of 30 values

    if orientation:
        vect = [0, 0, ((2*dist)/length), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        for i in opendog.pinFrames:
            X0=np.hstack((X0, opendog.getFrameXYZRPY(i)))
            #X0.extend(p.getLinkState(opendog.id,i)[:2][0]) #pybullet
            #X0.extend(p.getLinkState(opendog.id,i)[:2][1][:-1])
        
    else:
        vect = [0, 0, ((2*dist)/length), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        for frame in opendog.pinFrames:
            X0 = np.hstack((X0, opendog.data.oMf[frame].translation))
            #X0.extend(p.getLinkState(opendog.id,i)[0])

    traj = [X0]
    for i in range(length//2):
        next_X = list(map(operator.sub, traj[-1], vect))
        traj.append(next_X)
    for j in range(length//2):
        next_X = list(map(operator.add, traj[-1], vect))
        traj.append(next_X)

    return traj

#opendog=OpenDog()
#print(trajectoryUpAndDown(opendog))

# def ftraj(t, x0, z0):  # arguments : time, initial position x and z
#     """
#     from : https://github.com/EtienneAr/ISAE-PIE-quadruped-solo_pybullet/blob/master/solo_pybullet/controller.py
#     """
#     global T, dx, dz
#     x = []
#     z = []
#     if t >= T:
#         t %= T
#     x.append(x0 - dx * np.cos(2 * np.pi * t / T))
#     if t <= T / 2.:
#         z.append(z0 + dz * np.sin(2 * np.pi * t / T))
#     else:
#         z.append(0)
#     return np.matrix([x, z])