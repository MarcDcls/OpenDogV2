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
#either expressed or implied, of the Pinocchio project
import pinocchio as pin
import numpy as np

# a tester dans simulation

##
## In this short script, we show how to compute the derivatives of the
## inverse dynamics (RNEA), using the algorithms proposed in:
##
## Analytical Derivatives of Rigid Body Dynamics Algorithms, Justin Carpentier and Nicolas Mansard, Robotics: Science and Systems, 2018
##

# Create model and data

model = pin.buildSampleModelHumanoidRandom()
data = model.createData()

# Set bounds (by default they are undefinded for a the Simple Humanoid model)

model.lowerPositionLimit = -np.ones((model.nq,1))
model.upperPositionLimit = np.ones((model.nq,1))

q = pin.randomConfiguration(model) # joint configuration
v = np.random.rand(model.nv,1) # joint velocity
a = np.random.rand(model.nv,1) # joint acceleration

# Evaluate the derivatives

pin.computeRNEADerivatives(model,data,q,v,a)

# Retrieve the derivatives in data

dtau_dq = data.dtau_dq # Derivatives of the ID w.r.t. the joint config vector
dtau_dv = data.dtau_dv # Derivatives of the ID w.r.t. the joint velocity vector
dtau_da = data.M # Derivatives of the ID w.r.t. the joint acceleration vector
