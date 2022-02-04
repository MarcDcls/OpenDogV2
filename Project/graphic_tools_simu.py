import pybullet as p

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
            p.addUserDebugLine(polygon[i], polygon[0], lineColorRGB=color, lineWidth=10, lifeTime=durationTime)
        else:
            p.addUserDebugLine(polygon[i], polygon[i+1], lineColorRGB=color,lineWidth=10, lifeTime=durationTime)