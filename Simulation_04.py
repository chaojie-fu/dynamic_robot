import pybullet as p
import time
import pybullet_data
import numpy as np
import math
physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -10)
p.setPhysicsEngineParameter(numSolverIterations=50)
p.setTimeOut(40000000)
TimeStep = 1./100
p.setTimeStep(TimeStep)

planeId = p.loadURDF("Plane_Model/plane.urdf")
robotStartPos = [0, 0, 0.5]
robotStartOrientation = p.getQuaternionFromEuler([0,0,0])
robotId = p.loadURDF("robot.urdf", robotStartPos, robotStartOrientation)

nJoints = p.getNumJoints(robotId)
jointNameToId = {}
for j in range(nJoints):
    jointInfo = p.getJointInfo(robotId, j)
    jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]

if (True):
    bodyLF = jointNameToId['bodyLF']
    bodyRF = jointNameToId['bodyRF']
    bodyLB = jointNameToId['bodyLB']
    bodyRB = jointNameToId['bodyRB']
    hipLF = jointNameToId['hipLF']
    hipRF = jointNameToId['hipRF']
    hipLB = jointNameToId['hipLB']
    hipRB = jointNameToId['hipRB']
    kneeLF = jointNameToId['kneeLF']
    kneeRF = jointNameToId['kneeRF']
    kneeLB = jointNameToId['kneeLB']
    kneeRB = jointNameToId['kneeRB']

legnumbering = [
    bodyLF, bodyRF, bodyLB, bodyRB,
    hipLF, hipRF, hipLB, hipRB,
    kneeLF, kneeRF, kneeLB, kneeRB
]

halfpi = 1.57079632679
twopi = 4 * halfpi
pi = 2 * halfpi
motordir = [halfpi, -halfpi, -halfpi, halfpi,
            halfpi, -halfpi, halfpi, -halfpi,
            -halfpi, halfpi, -halfpi, halfpi
]

para = [
    0.2, 0.2, 0.2, 0.2,
    0, 0, 0, 0,
    0, 0, 0, 0
]

control = np.multiply(motordir, para)

# Start Initialization
for i in range(12):
    p.setJointMotorControl2(
        bodyIndex=robotId,
        jointIndex=legnumbering[i],
        controlMode=p.POSITION_CONTROL,
        targetPosition=control[i],
        # positionGain=0.1,
        # velocityGain=0.5,
        force=100
    )
# End Initialization

# Start Simulation
velocity_leg = 1
velocity_knee = 1
limit_leg = 0.9
t1 = 0
t2 = 0.3
for i in range (10000):
    if velocity_leg * t1 >= limit_leg:
        t1 = 0
    para = [
        0.2, 0.2, 0.2, 0.2,
        -velocity_leg * t1, -velocity_leg * t1, velocity_leg * t1, velocity_leg * t1,
        -0.3, -0.3, 0.3, 0.3
    ]
    control_F = np.multiply(motordir, para)
    p.setJointMotorControl2(
        bodyIndex=robotId,
        jointIndex=legnumbering[4],
        controlMode=p.POSITION_CONTROL,
        targetPosition=control_F[4],
        # positionGain=0.1,
        # velocityGain=0.5,
        force=100
    )
    p.setJointMotorControl2(
        bodyIndex=robotId,
        jointIndex=legnumbering[5],
        controlMode=p.POSITION_CONTROL,
        targetPosition=control_F[5],
        # positionGain=0.1,
        # velocityGain=0.5,
        force=100
    )
    p.setJointMotorControl2(
        bodyIndex=robotId,
        jointIndex=legnumbering[6],
        controlMode=p.POSITION_CONTROL,
        targetPosition=control_F[6],
        # positionGain=0.1,
        # velocityGain=0.5,
        force=100
    )
    p.setJointMotorControl2(
        bodyIndex=robotId,
        jointIndex=legnumbering[7],
        controlMode=p.POSITION_CONTROL,
        targetPosition=control_F[7],
        # positionGain=0.1,
        # velocityGain=0.5,
        force=100
    )
    p.setJointMotorControl2(
        bodyIndex=robotId,
        jointIndex=legnumbering[8],
        controlMode=p.POSITION_CONTROL,
        targetPosition=control_F[8],
        # positionGain=0.1,
        # velocityGain=0.5,
        force=100
    )
    p.setJointMotorControl2(
        bodyIndex=robotId,
        jointIndex=legnumbering[9],
        controlMode=p.POSITION_CONTROL,
        targetPosition=control_F[9],
        # positionGain=0.1,
        # velocityGain=0.5,
        force=100
    )
    p.setJointMotorControl2(
        bodyIndex=robotId,
        jointIndex=legnumbering[10],
        controlMode=p.POSITION_CONTROL,
        targetPosition=control_F[10],
        # positionGain=0.1,
        # velocityGain=0.5,
        force=100
    )
    p.setJointMotorControl2(
        bodyIndex=robotId,
        jointIndex=legnumbering[11],
        controlMode=p.POSITION_CONTROL,
        targetPosition=control_F[11],
        # positionGain=0.1,
        # velocityGain=0.5,
        force=100
    )
    p.stepSimulation()
    t1 = t1 + TimeStep
    time.sleep(TimeStep)
# End Simulation
p.disconnect()