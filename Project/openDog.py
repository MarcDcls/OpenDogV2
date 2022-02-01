import pybullet as p
import pinocchio as pin

from customExecption import *
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

    def getFootCartesianPosition(self, idLeg) :
        """
        Return the cartesian position of a given foot

        :param: leg identifiant as 'bl', 'br', 'fl' or 'fr'
        """
        try :
            if idLeg == "bl" or idLeg == 3 :
                return p.getJointState(self.id,12)[0]
            elif idLeg == "br" or idLeg == 4 :
                return p.getJointState(self.id,16)[0]
            elif idLeg == "fl" or idLeg == 1 :
                return p.getJointState(self.id,4)[0]
            elif idLeg == "fr"  or idLeg == 2 :
                return p.getJointState(self.id,8)[0]
            else :
                raise InputNotRecognizedAsLeg
        except InputNotRecognizedAsLeg : 
            print('Leg not identified, movement impossible')

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

    def walk(self,q,qdot,dt):
        distance=qdot/dt
        footCartesianPosition = []
        for i in ["fl","fr","bl","br"] :
           footCartesianPosition.append(self.getFootCartesianPosition(i))
        
        target = distance + footCartesianPosition
        
        q_calculated=[]


        return q_calculated
