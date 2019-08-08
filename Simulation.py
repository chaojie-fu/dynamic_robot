import pybullet as p
import time
physicsClinet = p.connect(p.GUI)
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")
robotStartPos = [0,0,1]
robotId = p.loadURDF("robot.urdf", robotStartPos)

