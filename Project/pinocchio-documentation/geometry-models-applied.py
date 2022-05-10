#BSD 2-Clause License

#Copyright (c) 2014-2021, CNRS 
#Copyright (c) 2018-2021, INRIA
#All rights reserved.

#Redistribution and use in source and binary forms, with or without
#modification, are permitted provided that the following conditions are met:

#1. Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#2. Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.

#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
#ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
#WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
#ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
#(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
#ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#The views and conclusions contained in the software and documentation are those
#of the authors and should not be interpreted as representing official policies,
#either expressed or implied, of the Pinocchio project.

import pinocchio
from sys import argv
from os.path import dirname, join, abspath


# This path refers to Pinocchio source code but you can define your own directory here.
#pinocchio_model_dir = join(dirname(dirname(str(abspath(__file__)))), "urdf")
pinocchio_model_dir = "home/ajonc/OpenDogV2/Project/urdf"
print(pinocchio_model_dir)
#model_path = join(pinocchio_model_dir,"example-robot-data/robots") if len(argv)<2 else argv[1]
mesh_dir = pinocchio_model_dir
urdf_model_path = join(pinocchio_model_dir,"robot.urdf")

#print(urdf_model_path)

# Load the urdf model
model, collision_model, visual_model = pinocchio.buildModelsFromUrdf(urdf_model_path, mesh_dir)
print('model name: ' + model.name)

# Create data required by the algorithms
data, collision_data, visual_data  = pinocchio.createDatas(model, collision_model, visual_model)

# Sample a random configuration
q        = pinocchio.randomConfiguration(model)
print('q: %s' % q.T)

# Perform the forward kinematics over the kinematic tree
pinocchio.forwardKinematics(model,data,q)

# Update Geometry models
pinocchio.updateGeometryPlacements(model, data, collision_model, collision_data)
pinocchio.updateGeometryPlacements(model, data, visual_model, visual_data)

# Print out the placement of each joint of the kinematic tree
print("\nJoint placements:")
for name, oMi in zip(model.names, data.oMi):
    print(("{:<24} : {: .2f} {: .2f} {: .2f}"
          .format( name, *oMi.translation.T.flat )))

# Print out the placement of each collision geometry object
print("\nCollision object placements:")
for k, oMg in enumerate(collision_data.oMg):
    print(("{:d} : {: .2f} {: .2f} {: .2f}"
          .format( k, *oMg.translation.T.flat )))

# Print out the placement of each visual geometry object
print("\nVisual object placements:")
for k, oMg in enumerate(visual_data.oMg):
    print(("{:d} : {: .2f} {: .2f} {: .2f}"
          .format( k, *oMg.translation.T.flat )))
