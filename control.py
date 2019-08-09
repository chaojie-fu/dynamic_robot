import pybullet as p
import time
import pybullet_data
import numpy as np
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
p.setPhysicsEngineParameter(numSolverIterations=50)
p.setTimeOut(40000000)
p.setTimeStep(1./100)

planeId = p.loadURDF("plane.urdf")
robotStartPos = [0, 0, 0.5]
robotStartOrientation = p.getQuaternionFromEuler([0,0,0])
robotId = p.loadURDF("robot.urdf", robotStartPos, robotStartOrientation)

nJoints = p.getNumJoints(robotId)
jointNameToId = {}
for j in range(nJoints):
    jointInfo = p.getJointInfo(robotId, j)
    jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]

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
    0, 0, 0, 0,
    0, 0, 0, 0,
    1, 1, 1, 1,
]

control = np.multiply(motordir,para)

#initialize
for i in range(12):
    p.setJointMotorControl2(
        bodyIndex=robotId,
        jointIndex=legnumbering[i],
        controlMode=p.POSITION_CONTROL,
        targetPosition=control[i],
        positionGain=1,
        velocityGain=0.5,
        force=10
    )

for i in range (10000):

    p.stepSimulation()
    time.sleep(1./100.)
p.disconnect()