import pybullet as p
import time
import pybullet_data
import matplotlib.pyplot as plt
import numpy as np

# q -- current angle value
# qRef -- desired angle value
# kp -- proportional gain
def velocityProportionalPositionReg(q, qRef, kp):
    e = q - qRef
    vel = -kp * e
    return vel

# dq -- current angular velocity value
# dqRef -- desired angular velocity value
# kp -- proportional gain
def torqueProportionalIntegralDerivativeVelocityReg(dq, dqRef, kp, ki, eInt, dt, ePrev):
    e = dq - dqRef
    eInt += e * dt
    de = (e - ePrev)
    tau = -kp * e - ki * eInt - kd * de
    ePrev = e
    return tau, eInt, ePrev


# dq -- current angular velocity value
# dqRef -- desired angular velocity value
# kp -- proportional gain
def torqueProportionalIntegralDerivativePositionReg(q, qRef, kp, ki, eInt, dt, ePrev):
    e = q - qRef
    eInt += e * dt
    de = (e - ePrev) / dt
    tau = -kp * e - ki * eInt - kd * de
    ePrev = e
    return tau, eInt, ePrev


PB_GUI = False

dt = 1/240   # pybullet simulation step
q0 = 0.0     # starting position (rad)
dq0 = 0.0    # starting velocity (rad/sec)
qRef = 0.5   # desired position (rad)
dqRef = 0.5  # desired velocity (rad/sec)
kp = 80.0
ki = 80.0
kd = 40.0
eInt = 0.0  # error integral
ePrev = 0.0

maxTime = 5.0
logTime = np.arange(0.0, maxTime, dt)
sz = len(logTime)
logAngle = np.zeros(sz)
logVel   = np.zeros(sz)
logTau   = np.zeros(sz)

physicsClient = p.connect(p.GUI if PB_GUI else p.DIRECT) # or p.DIRECT for non-graphical version
p.setGravity(0,0,-10)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")
pendulumId = p.loadURDF("./simple.urdf.xml", useFixedBase=True)

# get rid of all the default damping forces
p.changeDynamics(pendulumId, 1, linearDamping=0, angularDamping=0)
p.changeDynamics(pendulumId, 2, linearDamping=0, angularDamping=0)

# go to the starting position
p.setJointMotorControl2(bodyIndex=pendulumId, jointIndex=1, targetPosition=q0, controlMode=p.POSITION_CONTROL)
for _ in range(1000):
    p.stepSimulation()

# turn off the motor for the free motion
p.setJointMotorControl2(bodyIndex=pendulumId, jointIndex=1, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)
for idx in range(sz):
    p.stepSimulation()
    q  = p.getJointState(pendulumId, 1)[0]
    dq = p.getJointState(pendulumId, 1)[1]
    logAngle[idx] = q
    logVel[idx]   = dq

    # # Cascade two-level controller
    # # 1 Compute velocity neccessary to move to desired angle position
    # dqRef = velocityProportionalPositionReg(q, qRef, kp)
    # # logVel[idx] = vel
    # # p.setJointMotorControl2(bodyIndex=pendulumId, jointIndex=1, targetVelocity=vel, controlMode=p.VELOCITY_CONTROL)

    # # 2 Compute torque neccessary to provide computed velocity
    # tau, eInt, ePrev = torqueProportionalIntegralDerivativeVelocityReg(dq, dqRef, kp, ki, eInt, dt, ePrev)
    # logTau[idx] = tau
    # p.setJointMotorControl2(bodyIndex=pendulumId, jointIndex=1, force=tau, controlMode=p.TORQUE_CONTROL)

    tau, eInt, ePrev = torqueProportionalIntegralDerivativePositionReg(q, qRef, kp, ki, eInt, dt, ePrev)
    logTau[idx] = tau
    p.setJointMotorControl2(bodyIndex=pendulumId, jointIndex=1, force=tau, controlMode=p.TORQUE_CONTROL)

    if PB_GUI:
        time.sleep(dt)
p.disconnect()

plt.subplot(3,1,1)
plt.plot(logTime, logAngle, label='Angle')
plt.plot([logTime[0], logTime[-1]], [qRef, qRef], 'r--', label='RefVal')
plt.grid(True)
plt.legend()

plt.subplot(3,1,2)
plt.plot(logTime, logVel, label='Vel')
# plt.plot([logTime[0], logTime[-1]], [dqRef, dqRef], 'r--', label='RefVel')
plt.grid(True)
plt.legend()

plt.subplot(3,1,3)
plt.plot(logTime, logTau, label='Torque')
plt.grid(True)
plt.legend()

plt.show()
