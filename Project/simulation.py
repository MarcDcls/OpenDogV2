import time

import math
import numpy as np
import pybullet as p
import pybullet_data

from customExecption import *
from calculation import *
from ray_casting_algorithm import *

class OpenDog:

    def __init__(
            self,
    ):

        self.id = p.loadURDF("urdf/robot.urdf", [0, 0, 0.6], p.getQuaternionFromEuler([0, 0, 0]))
        self.legs = [[0, 1, 2], [3, 4, 5], [6, 7, 8], [9, 10, 11]]
        self._buildJointNameToIdDict()

    def _buildJointNameToIdDict(self):
        num_joints = p.getNumJoints(self.id)
        self.joint_name_to_id = {}
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.id, i)
            self.joint_name_to_id[joint_info[1].decode("UTF-8")] = joint_info[0]

    # def moveCylinders(self, V, force):
    #     for i in range(12):
    #         if 0.4455 - V[i] * 0.001 > -0.0044999 or 0.4455 - V[i] * 0.001 < -0.2045001:
    #             print("ERROR !!! Le verin ", i, "est hors limites : ",  0.4455 - V[i] * 0.001)
    #     p.setJointMotorControl2(self.id, self.joint_name_to_id['0v1'], targetPosition=0.4455 - V[0] * 0.001,
    #                             controlMode=p.POSITION_CONTROL, force=force)
    #     p.setJointMotorControl2(self.id, self.joint_name_to_id['0v2'], targetPosition=0.4455 - V[1] * 0.001,
    #                             controlMode=p.POSITION_CONTROL, force=force)
    #     p.setJointMotorControl2(self.id, self.joint_name_to_id['0v3'], targetPosition=0.4455 - V[2] * 0.001,
    #                             controlMode=p.POSITION_CONTROL, force=force)
    #     p.setJointMotorControl2(self.id, self.joint_name_to_id['1v1'], targetPosition=0.4455 - V[3] * 0.001,
    #                             controlMode=p.POSITION_CONTROL, force=force)
    #     p.setJointMotorControl2(self.id, self.joint_name_to_id['1v2'], targetPosition=0.4455 - V[4] * 0.001,
    #                             controlMode=p.POSITION_CONTROL, force=force)
    #     p.setJointMotorControl2(self.id, self.joint_name_to_id['1v3'], targetPosition=0.4455 - V[5] * 0.001,
    #                             controlMode=p.POSITION_CONTROL, force=force)
    #     p.setJointMotorControl2(self.id, self.joint_name_to_id['2v1'], targetPosition=0.4455 - V[6] * 0.001,
    #                             controlMode=p.POSITION_CONTROL, force=force)
    #     p.setJointMotorControl2(self.id, self.joint_name_to_id['2v2'], targetPosition=0.4455 - V[7] * 0.001,
    #                             controlMode=p.POSITION_CONTROL, force=force)
    #     p.setJointMotorControl2(self.id, self.joint_name_to_id['2v3'], targetPosition=0.4455 - V[8] * 0.001,
    #                             controlMode=p.POSITION_CONTROL, force=force)
    #     p.setJointMotorControl2(self.id, self.joint_name_to_id['3v1'], targetPosition=0.4455 - V[9] * 0.001,
    #                             controlMode=p.POSITION_CONTROL, force=force)
    #     p.setJointMotorControl2(self.id, self.joint_name_to_id['3v2'], targetPosition=0.4455 - V[10] * 0.001,
    #                             controlMode=p.POSITION_CONTROL, force=force)
    #     p.setJointMotorControl2(self.id, self.joint_name_to_id['3v3'], targetPosition=0.4455 - V[11] * 0.001,
    #                             controlMode=p.POSITION_CONTROL, force=force)

    # def isOverTurn(self) :
    #     '''
    #     Check if the robot is overturned

    #     :return: Boolean, robot state
    #     '''
    #     overturned = False
    #     for i in [[1,4],[5,8],[9,12], [13,16]]:
    #         if p.getLinkState(self.id,i[1])[0][2]>p.getLinkState(self.id,i[0])[0][2]:
    #             print(p.getLinkState(self.id,i[1])[0][2],p.getLinkState(self.id,i[0])[0][2])
    #             overturned=True
    #     return overturned

    def getCOM(self):
        """
        Computes OpenDog's center of mass coordinates

        :return: center of mass coordinates
        """
        nb_links = p.getNumJoints(self.id)
        base_pos = p.getBasePositionAndOrientation(self.id)[0]
        base_mass = p.getDynamicsInfo(self.id, -1)[0]
        com = [base_pos[0] * base_mass, base_pos[1] * base_mass, base_pos[2] * base_mass]
        mass = base_mass
        for i in range(nb_links):
            link_pos = p.getLinkState(self.id, i)[0]
            link_mass = p.getDynamicsInfo(self.id, i)[0]
            com = [com[0] + link_pos[0] * link_mass, com[1] + link_pos[1] * link_mass, com[2] + link_pos[2] * link_mass]
            mass += link_mass
        return [com[0] / mass, com[1] / mass, com[2] / mass]
    
    def getSupportPolygon(self):
        """
        Computes OpenDog's support polygon

        :return: support polygon coordinates
        :return: center of support polygon coordinates
        """
        coordinates = []
    
        nb_links = p.getNumJoints(self.id)
        for i in range(nb_links):
            link_pos = p.getLinkState(self.id, i)[0]

            if i % 4 == 0 and i != 0: # check if link_pos[i] is a foot
                if link_pos[2] < 0.038 and link_pos[2] > 0.034 : #check if the foot is on the ground
                    coordinates.append(link_pos)
        
        if len(coordinates)==4:
            coordinates=[coordinates[0],coordinates[1],coordinates[-1],coordinates[2]]
        
        return coordinates

    def getLegAngularPositions(self, idLeg):
        """
        Return the angular position of a given leg

        :param: leg identifiant as 'bl', 'br', 'fl' or 'fr'
        """
        try :
            if idLeg == "bl" or idLeg == 3 :
                return [p.getJointState(self.id,9)[0],p.getJointState(self.id,10)[0],p.getJointState(self.id,11)[0] ]
            elif idLeg == "br" or idLeg == 4 :
                return [p.getJointState(self.id,13)[0],p.getJointState(self.id,14)[0],p.getJointState(self.id,15)[0] ]
            elif idLeg == "fl" or idLeg == 1 :
                return [p.getJointState(self.id,1)[0],p.getJointState(self.id,2)[0],p.getJointState(self.id,3)[0] ]
            elif idLeg == "fr"  or idLeg == 2 :
                return [p.getJointState(self.id,5)[0],p.getJointState(self.id,6)[0],p.getJointState(self.id,7)[0] ]
            else :
                raise InputNotRecognizedAsLeg
        except InputNotRecognizedAsLeg : 
            print('Leg not identified, movement impossible')

    def getJointAngularPosition(self, idLeg, idJoint):
        """
        Return the angular position of a given leg

        :param: leg identifiant as 'bl', 'br', 'fl' or 'fr'
        :param: leg identifiant as 'hip', 'knee, 'ankle'
        """
        try :
            if idLeg == "fl"  or idLeg == 1 or idLeg == "fr" or idLeg == 2 or idLeg == "bl" or idLeg == 3 or idLeg == "br" or idLeg == 4  :
                if idJoint=="hip":
                    return self.getLegAngularPositions(idLeg)[0]
                elif idJoint=="knee":
                    return self.getLegAngularPositions(idLeg)[1]
                elif idJoint=="ankle":
                    return self.getLegAngularPositions(idLeg)[2]
                else :
                    raise InputNotRecognizedAsJoint
            else :
                raise InputNotRecognizedAsLeg
        except InputNotRecognizedAsLeg : 
            print('Leg not identified, movement impossible')
        except InputNotRecognizedAsJoint :
            print('Joint not identified, movement impossible')
    
    def moveAllLegs(self, q, force=1000):
        self.moveLeg(1,q[0],q[1],q[2],force)
        self.moveLeg(2,q[3],q[4],q[5],force)
        self.moveLeg(3,q[6],q[7],q[8],force)
        self.moveLeg(4,q[9],q[10],q[11],force)

    def moveLeg(self, idLeg, angularPositionHip=None, angularPositionKnee=None, angularPositionAnkle=None, force=1000):
        """
        Allow leg movement for given angular position 

        :param: leg identifiant as 'bl', 'br', 'fl' or 'fr'
        :param: desired hip angle position, None for no change
        :param: desired knee angle position , None for no change
        :param: desired ankle angle position
        """
        try :
            # evaluer les sorties du range ?
            angularPositionHip = self.getJointAngularPosition(idLeg,"hip") if (angularPositionHip==None) else angularPositionHip
            angularPositionKnee = self.getJointAngularPosition(idLeg,"knee") if (angularPositionKnee==None) else angularPositionKnee
            angularPositionAnkle = self.getJointAngularPosition(idLeg,"ankle") if (angularPositionAnkle==None) else angularPositionAnkle

            if idLeg == "fl" or idLeg == 1 :
                p.setJointMotorControl2(self.id, 1, targetPosition=angularPositionHip, controlMode=p.POSITION_CONTROL, force=force)
                p.setJointMotorControl2(self.id, 2, targetPosition=angularPositionKnee, controlMode=p.POSITION_CONTROL, force=force)
                p.setJointMotorControl2(self.id, 3, targetPosition=angularPositionAnkle, controlMode=p.POSITION_CONTROL, force=force)

            elif idLeg == "fr"  or idLeg == 2 :
                p.setJointMotorControl2(self.id, 5, targetPosition=angularPositionHip, controlMode=p.POSITION_CONTROL, force=force)
                p.setJointMotorControl2(self.id, 6, targetPosition=angularPositionKnee, controlMode=p.POSITION_CONTROL, force=force)
                p.setJointMotorControl2(self.id, 7, targetPosition=angularPositionAnkle, controlMode=p.POSITION_CONTROL, force=force)

            elif idLeg == "bl" or idLeg == 3 :
                p.setJointMotorControl2(self.id, 9, targetPosition=angularPositionHip, controlMode=p.POSITION_CONTROL, force=force)
                p.setJointMotorControl2(self.id, 10, targetPosition=angularPositionKnee, controlMode=p.POSITION_CONTROL, force=force)
                p.setJointMotorControl2(self.id, 11, targetPosition=angularPositionAnkle, controlMode=p.POSITION_CONTROL, force=force)

            elif idLeg == "br" or idLeg == 4 :
                p.setJointMotorControl2(self.id, 13, targetPosition=angularPositionHip, controlMode=p.POSITION_CONTROL, force=force)
                p.setJointMotorControl2(self.id, 14, targetPosition=angularPositionKnee, controlMode=p.POSITION_CONTROL, force=force)
                p.setJointMotorControl2(self.id, 15, targetPosition=angularPositionAnkle, controlMode=p.POSITION_CONTROL, force=force)
                
            else :
                raise InputNotRecognizedAsLeg

        except InputNotRecognizedAsLeg : 
            print('Leg not identified, movement impossible')
    
    def staticStability(self):
        """
        Return true is the center of mass is included in the support polygon, else false

        :param: center of mass
        :return: Boolean
        """
        com = self.getCOM()
        polygonCoordinates = self.getSupportPolygon() 
        included = False

        n=len(polygonCoordinates)
        if n>2 :
            #using ray casting algo
            included = ispointinside(com[:-1],polygonCoordinates)
        elif n==2 :
            ab=math.sqrt((polygonCoordinates[0][0] - polygonCoordinates[1][0])**2 + (polygonCoordinates[0][1] - polygonCoordinates[1][1])**2)
            ac=math.sqrt((polygonCoordinates[0][0] - com[0])**2 + (polygonCoordinates[0][1] - com[1])**2)
            cb=math.sqrt((com[0] - polygonCoordinates[1][0])**2 + (com[1] - polygonCoordinates[1][1])**2)
            if ac+cb==ab :
                included = True
        elif n==1 :
            if com[:-1] == coordinates[:-1]:
                included = True

        return included


def draw(pt, color=[1, 0, 0], durationTime=0):
    """
    Trace un point lors de la simulation

    :param pt: coordonnees du point
    :param color: couleur du point
    :param durationTime: duree en secondes d'affichage du point
    :return: None
    """
    end_pt = [pt[0]+0.0025, pt[1], pt[2]]
    p.addUserDebugLine(pt, end_pt, lineColorRGB=color, lineWidth=10, lifeTime=durationTime)

def drawHorizontalVector(pt, color=[0, 0, 1], durationTime=0):
    """
    Trace le vecteur horizontal lors de la simulation

    :param pt: coordonnees du point
    :param color: couleur du point
    :param durationTime: duree en secondes d'affichage du point
    :return: None
    """
    end_pt = [pt[0], pt[1], 0.0]
    p.addUserDebugLine(pt, end_pt, lineColorRGB=color, lineWidth=10, lifeTime=durationTime)


def drawPolygon(polygon, color=[0, 1, 0], durationTime=0):
    """
    Trace un polygone lors de la simulation

    :param pt: coordonnees de tous les points du polygone
    :param color: couleur du point
    :param durationTime: duree en secondes d'affichage du point
    :return: None
    """
    for i in range(len(polygon)):
        if i == (len(polygon)-1) :
            p.addUserDebugLine(polygon[i],polygon[0],lineColorRGB=color,lineWidth=10, lifeTime=durationTime)
        else :
            p.addUserDebugLine(polygon[i],polygon[i+1],lineColorRGB=color,lineWidth=10, lifeTime=durationTime)


################################ SIMULATION ################################

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")
p.setGravity(0, 0, -10)
opendog = OpenDog()



# p.changeVisualShape(opendog.id, -1, rgbaColor=[1, 1, 1, 1])#tronc
# p.changeVisualShape(opendog.id, 0, rgbaColor=[1, 1, 1, 1])#center frame
# p.changeVisualShape(opendog.id, 1, rgbaColor=[1, 0, 0, 1])#hip front left
# p.changeVisualShape(opendog.id, 2, rgbaColor=[1, 0, 0, 1])#upperleg front left
# p.changeVisualShape(opendog.id, 3, rgbaColor=[1, 0, 0, 1])#lowerleg front left
# p.changeVisualShape(opendog.id, 4, rgbaColor=[1, 0, 0, 1]) #foot_fl frame
# p.changeVisualShape(opendog.id, 5, rgbaColor=[0, 1, 0, 1])#mirrorhip back right
# p.changeVisualShape(opendog.id, 6, rgbaColor=[0, 1, 0, 1])#upperleg front right
# p.changeVisualShape(opendog.id, 7, rgbaColor=[0, 1, 0, 1])#lowerleg front right
# p.changeVisualShape(opendog.id, 8, rgbaColor=[0, 1, 0, 1])#foot_fr frame
# p.changeVisualShape(opendog.id, 9, rgbaColor=[0, 0, 1, 1])#mirrorhip back left
# p.changeVisualShape(opendog.id, 10, rgbaColor=[0, 0, 1, 1])#upperleg back left 
# p.changeVisualShape(opendog.id, 11, rgbaColor=[0, 0, 1, 1])#lowerleg back left
# p.changeVisualShape(opendog.id, 12, rgbaColor=[0, 0, 1, 1])#foot_bl frame
# p.changeVisualShape(opendog.id, 13, rgbaColor=[1, 1, 0, 1])#hip back right
# p.changeVisualShape(opendog.id, 14, rgbaColor=[1, 1, 0, 1])#upperleg back right 
# p.changeVisualShape(opendog.id, 15, rgbaColor=[1, 1, 0, 1])#lowerleg back right
# p.changeVisualShape(opendog.id, 16, rgbaColor=[1, 1, 0, 1])#foot_br frame



FRAME_RATE = 1 / 10 # Frequence a laquelle un ordre est transmis au verins
TICK_RATE = 1 / 240 # Frequence a laquelle le simulateur s'actualise

for joint in range(1,15): # motor up the joints
    p.setJointMotorControl2(opendog.id, joint, p.POSITION_CONTROL, targetPosition=0.01, force=1000, maxVelocity=3)

for i in range(10000):
    p.stepSimulation()
    
    # for i in range(p.getNumJoints(opendog.id)):
    #     print(i, ":", p.getLinkState(opendog.id, i)[0])

    # draw(opendog.getCOM())
    time.sleep(TICK_RATE)
    # if i == 495 :
    #     opendog.moveLeg("fl",0.5)
        # print(opendog.getLegAngularPositions("bl"))
    if i == 895 :
        drawHorizontalVector(opendog.getCOM())
        drawPolygon(opendog.getSupportPolygon())
    if i == 9999:
        print(opendog.isOverTurn())

p.disconnect()
