from custom_execption import *
from ray_casting_algorithm import *
import math


def getCOM(robot):
    """
    Computes robot's center of mass coordinates

    :param robot: robot object
    :return: center of mass coordinates
    """
    if robot.simu:
        nb_links = p.getNumJoints(robot.id)
        base_pos = p.getBasePositionAndOrientation(robot.id)[0]
        base_mass = p.getDynamicsInfo(robot.id, -1)[0]
        com = [base_pos[0] * base_mass, base_pos[1] * base_mass, base_pos[2] * base_mass]
        mass = base_mass
        for i in range(nb_links):
            link_pos = p.getLinkState(robot.id, i)[0]
            link_mass = p.getDynamicsInfo(robot.id, i)[0]
            com = [com[0] + link_pos[0] * link_mass, com[1] + link_pos[1] * link_mass, com[2] + link_pos[2] * link_mass]
            mass += link_mass
        return [com[0] / mass, com[1] / mass, com[2] / mass]
    else:
        return None  # To do : implement center of mass with Pinocchio ?


def getSupportPolygon(robot):
    """
    Computes OpenDog's support polygon

    :param robot: robot object
    :return: support polygon coordinates
    :return: center of support polygon coordinates
    """
    coordinates = []

    if robot.simu:
        nb_links = p.getNumJoints(robot.id)
        for i in range(nb_links):
            link_pos = p.getLinkState(robot.id, i)[0]

            if i % 4 == 0 and i != 0:  # check if link_pos[i] is a foot
                if link_pos[2] < 0.038 and link_pos[2] > 0.034:  # check if the foot is on the ground
                    coordinates.append(link_pos)

        if len(coordinates) == 4:
            coordinates = [coordinates[0], coordinates[1], coordinates[-1], coordinates[2]]
    else:
        pass  # To do : implement support polygon with Pinocchio ?
    return coordinates


def staticStability(robot):
    """
    Return true is the center of mass is included in the support polygon, else false

    :param robot: robot object
    :return: Boolean
    """
    com = getCOM(robot)
    polygonCoordinates = getSupportPolygon(robot)
    n = len(polygonCoordinates)
    included = False

    if n > 2:
        # using ray casting algo
        included = isPointInside(com[:-1], polygonCoordinates)
    elif n == 2:
        ab = math.sqrt((polygonCoordinates[0][0] - polygonCoordinates[1][0]) ** 2 + (
                polygonCoordinates[0][1] - polygonCoordinates[1][1]) ** 2)
        ac = math.sqrt((polygonCoordinates[0][0] - com[0]) ** 2 + (polygonCoordinates[0][1] - com[1]) ** 2)
        cb = math.sqrt((com[0] - polygonCoordinates[1][0]) ** 2 + (com[1] - polygonCoordinates[1][1]) ** 2)
        if ac + cb == ab:
            included = True
    elif n == 1:
        if com[:-1] == polygonCoordinates[:-1]:
            included = True

    return included


def getFrameCartesianPosition(robot, id, orientation=True):
    """
    Return the cartesian position of a given foot

    :param robot: robot object
    :param id: leg identifiant as 'bl', 'br', 'fl' or 'fr', 'cf'
    :param orientation: if true generate xyzrpy coordinates, else generate xyz coordinates
    """
    try:
        if robot.simu:
            if orientation:
                tmp = []
                if id == "bl" or id == 12:
                    tmp.extend(p.getLinkState(robot.id, 12)[:2][0])
                    tmp.extend(p.getLinkState(robot.id, 12)[:2][1][:-1])
                    return tmp
                elif id == "br" or id == 16:
                    tmp.extend(p.getLinkState(robot.id, 16)[:2][0])
                    tmp.extend(p.getLinkState(robot.id, 16)[:2][1][:-1])
                    return tmp
                elif id == "fl" or id == 4:
                    tmp.extend(p.getLinkState(robot.id, 4)[:2][0])
                    tmp.extend(p.getLinkState(robot.id, 4)[:2][1][:-1])
                    return tmp
                elif id == "fr" or id == 8:
                    tmp.extend(p.getLinkState(robot.id, 8)[:2][0])
                    tmp.extend(p.getLinkState(robot.id, 8)[:2][1][:-1])
                    return tmp
                elif id == "cf" or id == 0:
                    tmp.extend(p.getLinkState(robot.id, 0)[:2][0])
                    tmp.extend(p.getLinkState(robot.id, 0)[:2][1][:-1])
                    return tmp
                else:
                    raise InputNotRecognizedAsFrame
            else:
                if id == "bl" or id == 12:
                    return p.getLinkState(robot.id, 12)[0]
                elif id == "br" or id == 16:
                    return p.getLinkState(robot.id, 16)[0]
                elif id == "fl" or id == 4:
                    return p.getLinkState(robot.id, 4)[0]
                elif id == "fr" or id == 8:
                    return p.getLinkState(robot.id, 8)[0]
                elif id == "cf" or id == 0:
                    return p.getLinkState(robot.id, 0)[0]
                else:
                    raise InputNotRecognizedAsFrame
        else:
            if orientation:
                if id == "cf" or robot.pinFrames[0]:
                    return list(robot.getFrameXYZRPY(robot.pinFrames[0]))
                elif id == "fl" or robot.pinFrames[1]:
                    return list(robot.getFrameXYZRPY(robot.pinFrames[1]))
                elif id == "fr" or robot.pinFrames[2]:
                    return list(robot.getFrameXYZRPY(robot.pinFrames[2]))
                elif id == "bl" or robot.pinFrames[3]:
                    return list(robot.getFrameXYZRPY(robot.pinFrames[3]))
                elif id == "br" or robot.pinFrames[4]:
                    return list(robot.getFrameXYZRPY(robot.pinFrames[4]))
                else:
                    raise InputNotRecognizedAsFrame
            else:
                if id == "cf" or robot.pinFrames[0]:
                    return robot.data.oMf[robot.pinFrames[0]].translation
                elif id == "fl" or robot.pinFrames[1]:
                    return robot.data.oMf[robot.pinFrames[1]].translation
                elif id == "fr" or robot.pinFrames[2]:
                    return robot.data.oMf[robot.pinFrames[2]].translation
                elif id == "bl" or robot.pinFrames[3]:
                    return robot.data.oMf[robot.pinFrames[3]].translation
                elif id == "br" or robot.pinFrames[4]:
                    return robot.data.oMf[robot.pinFrames[4]].translation
                else:
                    raise InputNotRecognizedAsFrame

    except InputNotRecognizedAsLeg:
        print('Leg not identified, movement impossible')


def getJointAngularPosition(robot, jointName):
    """
    Return the angular position of a given joint

    :param jointName:  Name of the joint
    """
    if robot.simu:
        return p.getJointState(robot.id, robot.joint_name_to_id[jointName])[0]
    else:
        return None  # To do : Update with encoder values


def getLegAngularPositions(robot, idLeg):
    """
    Return the angular position of a given leg

    :param: leg identifiant as 'bl', 'br', 'fl' or 'fr'
    """
    try:
        if idLeg == "fl" or idLeg == "fr" or idLeg == "bl" or idLeg == "br":
            if robot.simu:
                return [getJointAngularPosition(robot, 'hip_' + idLeg),
                        getJointAngularPosition(robot, 'knee_' + idLeg),
                        getJointAngularPosition(robot, 'ankle_' + idLeg)]
            else:
                return None  # To do : Update with encoder values
        else:
            raise InputNotRecognizedAsLeg

    except InputNotRecognizedAsLeg:
        print('Leg not identified, movement impossible')


def getAngularPositionAllLegs(robot):
    pos = []
    for i in ['fl', 'fr', 'bl', 'br']:
        pos.append(getLegAngularPositions(robot, i))
    return pos
