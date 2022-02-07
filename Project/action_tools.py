import pybullet as p

from calculation_tools import getJointAngularPosition
from custom_execption import InputNotRecognizedAsLeg


def moveLeg(robot, idLeg, angularPositionHip=None, angularPositionKnee=None, angularPositionAnkle=None, force=1000):
    """
    Allow leg movement for given angular position 

    :param idLeg: leg identifiant as 'bl', 'br', 'fl' or 'fr'
    :param angularPositionHip: desired hip angle position, None for no change
    :param angularPositionKnee: desired knee angle position , None for no change
    :param angularPositionAnkle: desired ankle angle position
    """
    try:
        angularPositionHip = getJointAngularPosition(robot, 'hip_' + idLeg) if (
                angularPositionHip == None) else angularPositionHip
        angularPositionKnee = getJointAngularPosition(robot, 'knee_' + idLeg) if (
                angularPositionKnee == None) else angularPositionKnee
        angularPositionAnkle = getJointAngularPosition(robot, 'ankle_' + idLeg) if (
                angularPositionAnkle == None) else angularPositionAnkle

        if idLeg == "fl":
            p.setJointMotorControl2(robot.id, 1, targetPosition=angularPositionHip, controlMode=p.POSITION_CONTROL,
                                    force=force)
            p.setJointMotorControl2(robot.id, 2, targetPosition=angularPositionKnee, controlMode=p.POSITION_CONTROL,
                                    force=force)
            p.setJointMotorControl2(robot.id, 3, targetPosition=angularPositionAnkle, controlMode=p.POSITION_CONTROL,
                                    force=force)

        elif idLeg == "fr":
            p.setJointMotorControl2(robot.id, 5, targetPosition=angularPositionHip, controlMode=p.POSITION_CONTROL,
                                    force=force)
            p.setJointMotorControl2(robot.id, 6, targetPosition=angularPositionKnee, controlMode=p.POSITION_CONTROL,
                                    force=force)
            p.setJointMotorControl2(robot.id, 7, targetPosition=angularPositionAnkle, controlMode=p.POSITION_CONTROL,
                                    force=force)

        elif idLeg == "bl":
            p.setJointMotorControl2(robot.id, 9, targetPosition=angularPositionHip, controlMode=p.POSITION_CONTROL,
                                    force=force)
            p.setJointMotorControl2(robot.id, 10, targetPosition=angularPositionKnee, controlMode=p.POSITION_CONTROL,
                                    force=force)
            p.setJointMotorControl2(robot.id, 11, targetPosition=angularPositionAnkle, controlMode=p.POSITION_CONTROL,
                                    force=force)

        elif idLeg == "br":
            p.setJointMotorControl2(robot.id, 13, targetPosition=angularPositionHip, controlMode=p.POSITION_CONTROL,
                                    force=force)
            p.setJointMotorControl2(robot.id, 14, targetPosition=angularPositionKnee, controlMode=p.POSITION_CONTROL,
                                    force=force)
            p.setJointMotorControl2(robot.id, 15, targetPosition=angularPositionAnkle, controlMode=p.POSITION_CONTROL,
                                    force=force)

        else:
            raise InputNotRecognizedAsLeg

    except InputNotRecognizedAsLeg:
        print('Leg not identified, movement impossible')


def moveMotors(robot, q, force=1000):
    """
    Allow movement of all the motors of the robot

    :param robot: Robot object
    :param q: list of motors angular position
    :param force: force applied at each movement
    """
    moveLeg(robot, "fl", q[0], q[1], q[2], force)
    moveLeg(robot, "fr", q[3], q[4], q[5], force)
    moveLeg(robot, "bl", q[6], q[7], q[8], force)
    moveLeg(robot, "br", q[9], q[10], q[11], force)
