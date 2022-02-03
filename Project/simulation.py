import time

import math
import numpy as np
import pinocchio as pin
import pybullet as p
import pybullet_data

from openDog import OpenDog
from trajectory import trajectoryUpAndDown

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

# FRAME_RATE = 1 / 240 # Frequence a laquelle un ordre est transmis au verins
TICK_RATE = 1 / 240 # Frequence a laquelle le simulateur s'actualise

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")
p.setGravity(0, 0, -10)

opendog = OpenDog()
traj = trajectoryUpAndDown(opendog)


# Initialization time
for i in range(3):#3*TICK_RATE
    p.stepSimulation()  
    time.sleep(TICK_RATE)

# Simulation loop
for i in range(10000):
    p.stepSimulation()   

    for j in range(len(traj)):
        opendog.updateMotors()
        pin.forwardKinematics(opendog.model, opendog.data, opendog.motors)
        pin.updateFramePlacements(opendog.model, opendog.data) 

        X0 = []            
        for k in opendog.frames: # Marche mais récupère les données dans pybullet au lieu du modèle de pinocchio
            X0.extend(p.getLinkState(opendog.id, k)[:2][0])
            X0.extend(p.getLinkState(opendog.id, k)[:2][1][:-1])
        dX = np.subtract(traj[j], X0)

        
        J = pin.computeJointJacobians(opendog.model, opendog.data)
        print(J)
        dq = J.T @ dX
        q = opendog.motors + dq

        # J=[] # J = [ [J0], [J4], [J8], [J12], [J16]]
        # dq=[] # dq = [ [dq0], [dq4], [dq8], [dq12], [dq16]]
        # q=[] # q = [ q0, q4, q8, q12, q16]
        # i=0
        # for k in self.frames:
        #     J_tmp=pin.computeJointJacobian(self.model, self.data, res[-1], k)
        #     J.append(J_tmp)
        #     dq_tmp = J_tmp.T @ dX[6*i:6*(i+1)]
        #     dq.append(dq_tmp)
        #     i+=1
        #     new_q_frame = res[-1] + dq
        #     q.extend(new_q_frame)

    
    time.sleep(TICK_RATE)

p.disconnect()
