#from https://github.com/stack-of-tasks/pinocchio/blob/master/examples/meshcat-viewer.py
# pip install --user meshcat

import pinocchio 
import numpy 
import sys
import os
from os.path import dirname, join, abspath

from pinocchio.visualize import MeshcatVisualizer

# This path refers to Pinocchio source code but you can define your own directory here.
pinocchio_model_dir = join(dirname(dirname(str(abspath(__file__)))), "urdf")
mesh_dir = pinocchio_model_dir
urdf_model_path = join(pinocchio_model_dir,"robot.urdf")

#Load the urdf model
model, collision_model, visual_model = pinocchio.buildModelsFromUrdf(urdf_model_path, mesh_dir,pinocchio.JointModelFreeFlyer())

viz = MeshcatVisualizer(model, collision_model, visual_model)


# Start a new MeshCat server and client.
# Note: the server can also be started separately using the "meshcat-server" command in a terminal:
# this enables the server to remain active after the current script ends.
#
# Option open=True pens the visualizer.
# Note: the visualizer can also be opened seperately by visiting the provided URL.
try:
    viz.initViewer(open=True)
except ImportError as err:
    print("Error while initializing the viewer. It seems you should install Python meshcat")
    print(err)
    sys.exit(0)

# Load the robot in the viewer.
viz.loadViewerModel()


# Display a robot configuration.
q0 = pinocchio.neutral(model)
viz.display(q0)
viz.displayCollisions(True)
viz.displayVisuals(False)

mesh = visual_model.geometryObjects[0].geometry
mesh.buildConvexRepresentation(True)
convex = mesh.convex

if convex is not None:
    placement = pinocchio.SE3.Identity()
    placement.translation[0] = 2.
    geometry = pinocchio.GeometryObject("convex",0,convex,placement)
    geometry.meshColor = numpy.ones((4))
    visual_model.addGeometryObject(geometry)

# Display another robot.
viz2 = MeshcatVisualizer(model, collision_model, visual_model)
viz2.initViewer(viz.viewer)
viz2.loadViewerModel(rootNodeName = "pinocchio2")
q = q0.copy()
q[1] = 1.0
viz2.display(q)