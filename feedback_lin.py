import pybullet as p
import time
import pybullet_data
import numpy as np
from control.matlab import place

dt = 1/240 # pybullet simulation step
q0 = np.deg2rad(15)   # starting position (radian)
qd = 0.0

maxTime = 5
logTime = np.arange(0.0, maxTime, dt)
sz = len(logTime)
logPos = np.zeros(sz)
logPos[0] = q0
logVel = np.zeros(sz)
logCtrl = np.zeros(sz)
idx = 0

g = 10
L = 0.8
m = 1
kf = 1
a = -g/L
b = -1/(m*L*L)
c = kf*b

A = np.array([[0, 1],
              [0, 0]])
B = np.array([[0],
              [1]])
poles = np.array([-10,-20])
K = -place(A,B,poles)
print(f"poles theor: {poles}")
poles_fact = np.linalg.eig(A+B@K)
print(f"poles fact: {poles_fact}")

physicsClient = p.connect(p.DIRECT) # or p.DIRECT for non-graphical version
# p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-g)
# planeId = p.loadURDF("plane.urdf")
boxId = p.loadURDF("./simple.urdf.xml", useFixedBase=True)

# get rid of all the default damping forces
p.changeDynamics(boxId, 1, linearDamping=0, angularDamping=0)
p.changeDynamics(boxId, 2, linearDamping=0, angularDamping=0)

# go to the starting position
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetPosition=q0, controlMode=p.POSITION_CONTROL)
for _ in range(1000):
    p.stepSimulation()

# turn off the motor for the free motion
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)
for t in logTime[1:]:
    jointState = p.getJointState(boxId, 1)
    q = jointState[0]
    dq = jointState[1]

    e = q - qd

    # feedback linearization
    u = -K[0,0]*e - K[0,1]*dq
    v = (a*np.sin(q) + c*dq + u)/b

    p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, force=v, controlMode=p.TORQUE_CONTROL)
    p.stepSimulation()

    jointState = p.getJointState(boxId, 1)
    dq = jointState[1]
    logVel[idx] = dq
    logCtrl[idx] = v
    idx += 1
    logPos[idx] = q

logCtrl[-1] = v
p.disconnect()

import matplotlib.pyplot as plt

plt.subplot(3,1,1)
plt.grid(True)
plt.plot(logTime, logPos, label = "simPos")
plt.plot([logTime[0], logTime[-1]], [qd, qd], label = "refPos")
plt.legend()

plt.subplot(3,1,2)
plt.grid(True)
plt.plot(logTime, logVel, label = "simVel")
plt.legend()

plt.subplot(3,1,3)
plt.grid(True)
plt.plot(logTime, logCtrl, label = "simCtrl")
plt.legend()

plt.show()