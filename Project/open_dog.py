import numpy as np
from os.path import dirname, join, abspath
import pinocchio as pin
from pinocchio.rpy import matrixToRpy
import pybullet as p

from calculation_tools import *

class OpenDog:
    """
    Class used to represent OpenDog
    """

    def __init__(self, simu=True):
        if simu:
            self.id = p.loadURDF("urdf/robot.urdf", [0, 0, 0.6], p.getQuaternionFromEuler([0, 0, 0]))
            self._buildJointNameToIdDict()
        self.simu = simu
        self.model, self.collision_model, self.visual_model = self._generatePinocchioModels()
        self.data, self.collision_data, self.visual_data = pin.createDatas(self.model, self.collision_model,
                                                                           self.visual_model)
        self.URDFframes = [0, 4, 8, 12, 16]
        self.pinFrames = self._getPinocchioFrames()
        self.joints = [1, 2, 3, 5, 6, 7, 9, 10, 11, 13, 14, 15]
        self.motors = self._motorsInitialization()

    def _buildJointNameToIdDict(self):
        """
        Generate a dictionary matching the names of the urdf and the indices of the pybullet simulation
        """
        num_joints = p.getNumJoints(self.id)
        self.joint_name_to_id = {}
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.id, i)
            self.joint_name_to_id[joint_info[1].decode("UTF-8")] = joint_info[0]

    def _generatePinocchioModels(self):
        """
        Generate Pinocchio models
        """
        pinocchio_model_dir = join(dirname(dirname(str(abspath(__file__)))), "Project/urdf")
        mesh_dir = pinocchio_model_dir
        urdf_model_path = join(pinocchio_model_dir, "robot.urdf")

        return pin.buildModelsFromUrdf(urdf_model_path, mesh_dir)

    def _motorsInitialization(self):
        """
        Set and get initial values of the motors

        :return: motors angular position
        """
        q = pin.neutral(self.model)
        pin.forwardKinematics(self.model, self.data, q)
        if self.simu:
            for joint in self.joints:
                p.setJointMotorControl2(self.id, joint, p.POSITION_CONTROL, targetPosition=0, force=1000, maxVelocity=3)
        else:
            pass  # To do : set motors values
        return q

    def _getPinocchioFrames(self):
        """
        Get frames identifiant for pinocchio implementation
        """
        ID_CF = self.model.getFrameId("center_frame")
        ID_FL = self.model.getFrameId("foot_fl_frame")
        ID_FR = self.model.getFrameId("foot_fr_frame")
        ID_BL = self.model.getFrameId("foot_bl_frame")
        ID_BR = self.model.getFrameId("foot_br_frame")
        return [ID_CF, ID_FL, ID_FR, ID_BL, ID_BR]

    def updateMotors(self):
        """
        Get motors values
        """
        if self.simu:
            motors_list = getAngularPositionAllLegs(self)
            flat_list = [motor for motors in motors_list for motor in motors]
            self.motors = np.array(flat_list)
        else:
            self.motors = None  # to do : update with encoder values

    def getFrameXYZRPY(self, frameID):
        """
        Returns Cartesian coordinates of a given frame

        :param frameID: pinocchio frame identifiant
        :return: cartesian coordinates
        """
        return np.hstack((self.data.oMf[frameID].translation, matrixToRpy(self.data.oMf[frameID].rotation)))
