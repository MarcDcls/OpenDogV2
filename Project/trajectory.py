import numpy as np
import pinocchio as pin
import operator

def trajectoryUpAndDown(opendog, length=500, dist=0.2):
    """
    Generate a trajectory of cartesian coordinates

    :param qInit: initial joint configuration
    :param length: number of coodinates
    :param dist: heigth of the step / distance on z
    :return: List of cartesian coordinates
    """
    pin.forwardKinematics(opendog.model, opendog.data, opendog.motors)
    X0 = [] # vector of 30 values
    for i in opendog.frames:
        X0.extend(p.getLinkState(opendog.id,i)[:2][0])
        X0.extend(p.getLinkState(opendog.id,i)[:2][1][:-1])

    traj = [X0]
    vect = [0,0,((2*dist)/length),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    for i in range(length//2) :
        next_X = list(map(operator.sub, traj[-1], vect))
        traj.append(next_X)
    for j in range(length//2) :
        next_X = list(map(operator.add, traj[-1], vect))
        traj.append(next_X)

    return traj