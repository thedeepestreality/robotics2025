import pybullet as p
import numpy as np

dt = 1/240 # pybullet simulation step
q0 = 0.5 #np.deg2rad(15)   # starting position (radian)
qd = 0.0

maxTime = 5
logTime = np.arange(0.0, maxTime, dt)
sz = len(logTime)
logPos = np.zeros(sz)
logPos[0] = q0
logVel = np.zeros(sz)
idx = 0

ctrlTime = 2
ACC = -4 * (qd - q0) / ctrlTime**2

g = 10
L = 0.8
m = 1
kf = 1
a = -g/L
b = -1/(m*L*L)
c = kf*b

physicsClient = p.connect(p.DIRECT)
p.setGravity(0,0,-g)
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

    if t <= ctrlTime/2:
        u = ACC
    elif t <= ctrlTime:
        u = -ACC
    else:
        u = 0

    # feedback linearization
    tau = (a*np.sin(q) + c*dq + u)/b

    p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, force=tau, controlMode=p.TORQUE_CONTROL)
    p.stepSimulation()

    jointState = p.getJointState(boxId, 1)
    dq = jointState[1]
    logVel[idx] = dq
    idx += 1
    logPos[idx] = q

p.disconnect()

import matplotlib.pyplot as plt

plt.subplot(2,1,1)
plt.grid(True)
plt.plot(logTime, logPos, label = "simPos")
plt.plot([logTime[0], logTime[-1]], [qd, qd], label = "refPos")
plt.legend()

plt.subplot(2,1,2)
plt.grid(True)
plt.plot(logTime, logVel, label = "simVel")
plt.legend()

plt.show()