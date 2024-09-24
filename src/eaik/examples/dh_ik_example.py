import numpy as np

from eaik.IK_DH import Robot

"""
Example for a random robot kinematic
"""

d = np.array([0, 0, 0, 0.56426215, 0.31625527, 0])
alpha = np.array([np.pi/2, -np.pi/2, 0, np.pi/2, np.pi/2, 0])
a = np.array([0, 0.6766692, 0.93924826, 0.99652755, 0, 0.9355382])
bot = Robot(alpha, a, d)
print(bot.hasKnownDecomposition())
print(bot.fwdKin(np.array([1,1,1,1,1,1])))
