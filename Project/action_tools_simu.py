import pybullet as p

from calculation_tools_simu import getJointAngularPosition
from custom_execption import InputNotRecognizedAsLeg

joints = [1, 2, 3, 5, 6, 7, 9, 10, 11, 13, 14, 15]

def motorsInitialization(robotSimu):
    for joint in joints:
        p.setJointMotorControl2(robotSimu, joint, p.POSITION_CONTROL, targetPosition=0, force=1000, maxVelocity=3)

def updateMotors(robotSimu):
    m = []            
    for j in joints:
        m.append(p.getJointState(robotSimu, j)[0]) 
    #self.motors = np.array(m)

def moveLeg(robotSimu, idLeg, angularPositionHip=None, angularPositionKnee=None, angularPositionAnkle=None, force=1000):
    """
    Allow leg movement for given angular position 

    :param: leg identifiant as 'bl', 'br', 'fl' or 'fr'
    :param: desired hip angle position, None for no change
    :param: desired knee angle position , None for no change
    :param: desired ankle angle position
    """
    try :
        # evaluer les sorties du range ? 
        angularPositionHip = getJointAngularPosition(robotSimu,'hip_' + idLeg) if (angularPositionHip==None) else angularPositionHip
        angularPositionKnee = getJointAngularPosition(robotSimu,'knee_' + idLeg) if (angularPositionKnee==None) else angularPositionKnee
        angularPositionAnkle = getJointAngularPosition(robotSimu,'ankle_' + idLeg) if (angularPositionAnkle==None) else angularPositionAnkle

        if idLeg == "fl" :
            p.setJointMotorControl2(robotSimu, 1, targetPosition=angularPositionHip, controlMode=p.POSITION_CONTROL, force=force)
            p.setJointMotorControl2(robotSimu, 2, targetPosition=angularPositionKnee, controlMode=p.POSITION_CONTROL, force=force)
            p.setJointMotorControl2(robotSimu, 3, targetPosition=angularPositionAnkle, controlMode=p.POSITION_CONTROL, force=force)

        elif idLeg == "fr" :
            p.setJointMotorControl2(robotSimu, 5, targetPosition=angularPositionHip, controlMode=p.POSITION_CONTROL, force=force)
            p.setJointMotorControl2(robotSimu, 6, targetPosition=angularPositionKnee, controlMode=p.POSITION_CONTROL, force=force)
            p.setJointMotorControl2(robotSimu, 7, targetPosition=angularPositionAnkle, controlMode=p.POSITION_CONTROL, force=force)

        elif idLeg == "bl" :
            p.setJointMotorControl2(robotSimu, 9, targetPosition=angularPositionHip, controlMode=p.POSITION_CONTROL, force=force)
            p.setJointMotorControl2(robotSimu, 10, targetPosition=angularPositionKnee, controlMode=p.POSITION_CONTROL, force=force)
            p.setJointMotorControl2(robotSimu, 11, targetPosition=angularPositionAnkle, controlMode=p.POSITION_CONTROL, force=force)

        elif idLeg == "br" :
            p.setJointMotorControl2(robotSimu, 13, targetPosition=angularPositionHip, controlMode=p.POSITION_CONTROL, force=force)
            p.setJointMotorControl2(robotSimu, 14, targetPosition=angularPositionKnee, controlMode=p.POSITION_CONTROL, force=force)
            p.setJointMotorControl2(robotSimu, 15, targetPosition=angularPositionAnkle, controlMode=p.POSITION_CONTROL, force=force)
            
        else :
            raise InputNotRecognizedAsLeg

    except InputNotRecognizedAsLeg : 
        print('Leg not identified, movement impossible')


def moveMotors(robotSimu, q, force=1000):
    moveLeg(robotSimu, "fl", q[0], q[1], q[2], force)
    moveLeg(robotSimu, "fr", q[3], q[4], q[5], force)
    moveLeg(robotSimu, "bl", q[6], q[7], q[8], force)
    moveLeg(robotSimu, "br", q[9], q[10], q[11], force)

