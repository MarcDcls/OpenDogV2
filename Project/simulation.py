import time

import math
import numpy as np
import pybullet as p
import pybullet_data

from openDog import OpenDog

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

# for joint in range(1,15): # motor up the joints
#     p.setJointMotorControl2(opendog.id, joint, p.POSITION_CONTROL, targetPosition=0.01, force=1000, maxVelocity=3)

for joint in [1,9]:
    p.setJointMotorControl2(opendog.id, joint, p.POSITION_CONTROL, targetPosition=0.01, force=1000, maxVelocity=3)
for joint in [5,13]:
    p.setJointMotorControl2(opendog.id, joint, p.POSITION_CONTROL, targetPosition=-0.01, force=1000, maxVelocity=3)


for joint in [2,10]:
    p.setJointMotorControl2(opendog.id, joint, p.POSITION_CONTROL, targetPosition=0.32, force=1000, maxVelocity=3)
for joint in [6,14]:
    p.setJointMotorControl2(opendog.id, joint, p.POSITION_CONTROL, targetPosition=-0.32, force=1000, maxVelocity=3)


for joint in [3,11]:
    p.setJointMotorControl2(opendog.id, joint, p.POSITION_CONTROL, targetPosition=1.6, force=1000, maxVelocity=3)
for joint in [7,15]:
    p.setJointMotorControl2(opendog.id, joint, p.POSITION_CONTROL, targetPosition=-1.6, force=1000, maxVelocity=3)



for i in range(10000):
    p.stepSimulation()

    # for i in range(p.getNumJoints(opendog.id)):
    #     print(i, ":", p.getLinkState(opendog.id, i)[0])

    # draw(opendog.getCOM())
    time.sleep(TICK_RATE)

    if i == 495 :
        opendog.upDown()

    #     opendog.moveLeg("fl",0.5)
    #     # print(opendog.getLegAngularPositions("bl"))
    # if i == 895 :
    #     drawHorizontalVector(opendog.getCOM())
    #     drawPolygon(opendog.getSupportPolygon())

p.disconnect()
