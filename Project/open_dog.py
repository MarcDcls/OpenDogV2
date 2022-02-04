import numpy as np
import operator
import pinocchio as pin
from pinocchio.rpy import matrixToRpy
import pybullet as p
import time

from os.path import dirname, join, abspath

class OpenDog:
    """
    Class description
    """

    def __init__(self) :
        #self.id = p.loadURDF("urdf/robot.urdf", [0, 0, 0.6], p.getQuaternionFromEuler([0, 0, 0]))
        #self._buildJointNameToIdDict()
        self.model, self.collision_model, self.visual_model  = self._generatePinocchioModels()
        self.data, self.collision_data, self.visual_data = pin.createDatas(self.model, self.collision_model, self.visual_model)
        self.URDFframes = [0, 4, 8, 12, 16]
        self.pinFrames = self._setPinocchioFrames()
        self.joints = [1, 2, 3, 5, 6, 7, 9, 10, 11, 13, 14, 15]
        self.motors = self._motorsInitialization()
        #
        #self.com=self.com(self.motors)  # Compute the robot center of mass.


    def _buildJointNameToIdDict(self):
        num_joints = p.getNumJoints(self.id)
        self.joint_name_to_id = {}
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.id, i)
            self.joint_name_to_id[joint_info[1].decode("UTF-8")] = joint_info[0]
    
    def _generatePinocchioModels(self):
        pinocchio_model_dir = join(dirname(dirname(str(abspath(__file__)))), "Project/urdf")
        mesh_dir = pinocchio_model_dir
        urdf_model_path = join(pinocchio_model_dir,"robot.urdf")

        return pin.buildModelsFromUrdf(urdf_model_path,mesh_dir)

    def _motorsInitialization(self):
        q = pin.neutral(self.model)
        pin.forwardKinematics(self.model, self.data, q)
        return q
    
    def _setPinocchioFrames(self):
        ID_CF = self.model.getFrameId("center_frame")
        ID_FL = self.model.getFrameId("foot_fl_frame")
        ID_FR = self.model.getFrameId("foot_fr_frame")
        ID_BL = self.model.getFrameId("foot_bl_frame")
        ID_BR = self.model.getFrameId("foot_br_frame")
        return [ID_CF, ID_FL, ID_FR, ID_BL, ID_BR ]

    def updateMotors(self, q):
        # m = []            
        # for j in self.joints:
        #     m.append(p.getJointState(self.id, j)[0]) # a adapter à pinocchio
        #self.motors = np.array(m)
        self.motors = q

    def getFrameXYZRPY(self,frameID) :
        """
        Returns Cartesian coordinates of a given frame

        :param frameID: pinocchio frame identifiant
        :return: cartesian coordinates
        """
        return np.hstack((self.data.oMf[frameID].translation, matrixToRpy(self.data.oMf[frameID].rotation)))

    def move(self, framerate):
        """
        Return a list of angular positions in order to generate a movement

        :return: list of angular positions
        """
        #set initial joint configuration 
        legsPinocchioTree=["bl","br","fl","fr"]
        q=[]
        for i in legsPinocchioTree:
            q.extend(getLegAngularPositions(self,i))
        q=np.array(q)

        #set trajectory
        traj=trajectoryUpAndDown(self) #500 * 30

        #initialization of list of motor configuration : list of np.array ?
        res=[q]
        
        
        for i in range(len(traj)):
            pin.forwardKinematics(self.model, self.data,res[-1])
            pin.updateFramePlacements(self.model, self.data) 

            X0 = []            
            for j in self.frames:
                #mauvaise méthode ? mis à jour dans pinocchio mais pas dans pybullet
                #X0.extend(p.getLinkState(self.id,j)[0])
                X0.extend(p.getLinkState(self.id, j)[:2][0])
                X0.extend(p.getLinkState(self.id, j)[:2][1][:-1])
            dX=np.subtract(traj[j],X0)

            
            J=[] # J = [ [J0], [J4], [J8], [J12], [J16]]
            dq=[] # dq = [ [dq0], [dq4], [dq8], [dq12], [dq16]]
            q=[] # q = [ q0, q4, q8, q12, q16]
            i=0
            for k in self.frames:
                J_tmp=pin.computeJointJacobian(self.model, self.data, res[-1], k)
                J.append(J_tmp)
                dq_tmp = J_tmp.T @ dX[6*i:6*(i+1)]
                dq.append(dq_tmp)
                i+=1
                new_q_frame = res[-1] + dq
                q.extend(new_q_frame)

            res.append(q)
        
        return res

