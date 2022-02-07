from custom_execption import *
from ray_casting_algorithm import *
from trajectory import *
import math


def getCOM(robot):
    """
    Computes robot's center of mass coordinates

    :return: center of mass coordinates
    """
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


def getSupportPolygon(robot):
    """
    Computes OpenDog's support polygon

    :return: support polygon coordinates
    :return: center of support polygon coordinates
    """
    coordinates = []

    nb_links = p.getNumJoints(robot.id)
    for i in range(nb_links):
        link_pos = p.getLinkState(robot.id, i)[0]

        if i % 4 == 0 and i != 0:  # check if link_pos[i] is a foot
            if link_pos[2] < 0.038 and link_pos[2] > 0.034:  # check if the foot is on the ground
                coordinates.append(link_pos)

    if len(coordinates) == 4:
        coordinates = [coordinates[0], coordinates[1], coordinates[-1], coordinates[2]]

    return coordinates


def staticStability(robot):
    """
    Return true is the center of mass is included in the support polygon, else false

    :param: center of mass
    :return: Boolean
    """
    com = getCOM(robot)
    polygonCoordinates = getSupportPolygon(robot)
    included = False

    n = len(polygonCoordinates)
    if n > 2:
        # using ray casting algo
        included = ispointinside(com[:-1], polygonCoordinates)
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

    :param: leg identifiant as 'bl', 'br', 'fl' or 'fr', 'cf'
    """
    try:
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
                raise InputNotRecognizedAsLeg
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
                raise InputNotRecognizedAsLeg
    except InputNotRecognizedAsLeg:
        print('Leg not identified, movement impossible')


def getJointAngularPosition(robot, jointName):
    """
    Return the angular position of a given leg

    :param: leg identifiant as 'bl', 'br', 'fl' or 'fr'
    :param: leg identifiant as 'hip', 'knee, 'ankle'
    """
    if robot.simu:
        return p.getJointState(robot.id, robot.joint_name_to_id[jointName])[0]
    return None  # Dans le cas réel fournir les valeurs données par les encodeurs


def getLegAngularPositions(robot, idLeg):
    """
    Return the angular position of a given leg

    :param: leg identifiant as 'bl', 'br', 'fl' or 'fr'
    """
    try:
        if idLeg == "fl" or idLeg == "fr" or idLeg == "bl" or idLeg == "br":
            return [getJointAngularPosition(robot, 'hip_' + idLeg),
                    getJointAngularPosition(robot, 'knee_' + idLeg),
                    getJointAngularPosition(robot, 'ankle_' + idLeg)]
        else:
            raise InputNotRecognizedAsLeg

    except InputNotRecognizedAsLeg:
        print('Leg not identified, movement impossible')


def getAngularPositionAllLegs(robot):
    pos = []
    for i in ['fl', 'fr', 'bl', 'br']:
        pos.append(getLegAngularPositions(robot, i))
    return pos
