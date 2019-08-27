import pybullet as p
import time
import pybullet_data
import numpy as np
import math
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
p.setPhysicsEngineParameter(numSolverIterations=50)
p.setTimeOut(40000000)
TimeStep = 1./100
p.setTimeStep(TimeStep)

planeId = p.loadURDF("plane.urdf")
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
t = 0

for i in range (10000):
    if t <= 0.3:
        para = [
            0.2, 0.2, 0.2, 0.2,
            0+1.1*t, 0.33+1.1*t, 0.33+1.1*t, 0+1.1*t,
            0.33+1.1*t, 0.33+3.3*t, 0.33+3.3*t, 0.33+1.1*t
        ]
    elif t <= 0.6:
        para = [
            0.2, 0.2, 0.2, 0.2,
            0.33, 0.66-2.2*(t-0.3), 0.66-2.2*(t-0.3), 0.33,
            0.66-1.1*(t-0.3), 1.32-3.3*(t-0.3), 1.32-3.3*(t-0.3), 0.66-1.1*(t-0.3)
        ]
    elif t <= 0.9:
        para = [
            0.2, 0.2, 0.2, 0.2,
            0.33+1.1*(t-0.6), 0+1.1*(t-0.6), 0+1.1*(t-0.6), 0.33+1.1*(t-0.6),
            0.33+3.3*(t-0.6), 0.33+1.1*(t-0.6), 0.33+1.1*(t-0.6), 0.33+3.3*(t-0.6)
        ]
    elif t <= 1.2:
        para = [
            0.2, 0.2, 0.2, 0.2,
            0.66-2.2*(t-0.9), 0.33, 0.33, 0.66-2.2*(t-0.9),
            1.32-3.3*(t-0.9), 0.66-1.1*(t-0.9), 0.66-1.1*(t-0.9), 1.32-3.3*(t-0.9)
        ]
    else:
        t = 0

    control_F = np.multiply(motordir, para)*0.8
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
    t = t + TimeStep
    time.sleep(TimeStep)
# End Simulation
p.disconnect()