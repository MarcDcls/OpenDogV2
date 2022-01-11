import time

import pybullet as p
import pybullet_data

class OpenDog:
    """
    Links : self.legs[leg_id][i] = id du i-ème membre de la patte leg_id
    Joints : self.joint_name_to_id['ij'] = id du j-ème joint de la patte i
    """

    def __init__(
            self,
    ):
        self.id = p.loadURDF("urdf/basic_robot.urdf", [0, 0, 0.6], p.getQuaternionFromEuler([0, 0, 0]))
        # self.connectors = [[16, 8, 6, 9, 12, 17, 15, 72],
        #                    [34, 26, 24, 27, 30, 35, 33, 73],
        #                    [52, 44, 42, 45, 48, 53, 51, 74],
        #                    [70, 62, 60, 63, 66, 71, 69, 75]]
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
    #             print("ERROR !!! Le vérin ", i, "est hors limites : ",  0.4455 - V[i] * 0.001)
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
    #
    # def getCenterOfMassSimu(self):
    #     """
    #     Calcule les coordonnées du centre de masse du Megabot simulé à partir de l'URDF
    #     :return: coordonnées du centre de masse
    #     """
    #     nb_links = p.getNumJoints(self.id)
    #     base_pos = p.getBasePositionAndOrientation(self.id)[0]
    #     base_mass = p.getDynamicsInfo(self.id, -1)[0]
    #     com = [base_pos[0] * base_mass, base_pos[1] * base_mass, base_pos[2] * base_mass]
    #     mass = base_mass
    #     for i in range(nb_links):
    #         link_pos = p.getLinkState(self.id, i)[0]
    #         link_mass = p.getDynamicsInfo(self.id, i)[0]
    #         com = [com[0] + link_pos[0] * link_mass, com[1] + link_pos[1] * link_mass, com[2] + link_pos[2] * link_mass]
    #         mass += link_mass
    #     return [com[0] / mass, com[1] / mass, com[2] / mass]

    # def DrawCOM(self):
    #     p.addUserDebugLine(pl.get_leg_pos())


# def draw(pt, color=[1, 0, 0], durationTime=0):
#     """
#     Trace un point lors de la simulation
#     :param pt: coordonnées du point
#     :param color: couleur du point
#     :param durationTime: durée en secondes d'affichage du point
#     :return: None
#     """
#     end_pt = [pt[0]+0.0025, pt[1], pt[2]]
#     p.addUserDebugLine(pt, end_pt, lineColorRGB=color, lineWidth=10, lifeTime=durationTime)


################################ SIMULATION ################################

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")
p.setGravity(0, 0, -10)
opendog = OpenDog()

FRAME_RATE = 1 / 10 # Fréquence à laquelle un ordre est transmis au vérins
TICK_RATE = 1 / 240 # Fréquence à laquelle le simulateur s'actualise

for i in range(p.getNumJoints(opendog.id)):
    p.setDebugObjectColor(opendog.id, i, [1., 0., 0.])

for i in range(10000):
    p.stepSimulation()
    time.sleep(TICK_RATE)

p.disconnect()
