import pybullet as p
import time
import pybullet_data

dt = 1/240 # pybullet simulation step
q0 = 0.0   # starting position (radian)
qd = 0.5   # desired position (radian)
kp = 10.0
physicsClient = p.connect(p.GUI) # or p.DIRECT for non-graphical version
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
while True:
    p.stepSimulation()
    q = p.getJointState(pendulumId, 1)[0]
    e = q - qd
    vel = -kp * e
    p.setJointMotorControl2(bodyIndex=pendulumId, jointIndex=1, targetVelocity=vel, controlMode=p.VELOCITY_CONTROL)
    time.sleep(dt)
p.disconnect()