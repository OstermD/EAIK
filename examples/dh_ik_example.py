import numpy as np

from eaik.IK_DH import DhRobot

"""
Example for a random robot kinematic
"""

"""
# IRB
d = np.array([0.780, 0, 0, 1.142, 0, 0.2])
alpha = np.array([-np.pi/2, 0, -np.pi/2, -np.pi/2, np.pi/2, 0])
a = np.array([0.320, 1.28, 0.20, 0, 0, 0])
bot = DhRobot(alpha, a, d)
print(bot.hasKnownDecomposition())
print(bot.fwdKin(np.array([1,1,1,1,1,1])))

# Panda
d = np.array([0.333, 0, 0.316, 0, 0.384, 0, 0.107])
alpha = np.array([np.pi/2, -np.pi/2, -np.pi/2, np.pi/2, -np.pi/2, np.pi/2, 0])
a = np.array([0, 0, 0, 0.0825, -0.0825, 0, 0.088])
bot = DhRobot(alpha, a, d, [(6, 0)])
print(bot.hasKnownDecomposition())
print(bot.fwdKin(np.array([1,1,1,1,1,1,0])))

# Ur5
d = np.array([0.1273, 0, 0, 0.163941, 0.1157, 0.0922])
alpha = np.array([np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0])
a = np.array([0, -0.612, -0.573, 0, 0, 0])
bot = DhRobot(alpha, a, d)
print(bot.hasKnownDecomposition())
print(bot.fwdKin(np.array([1,1,1,1,1,1])))
"""

# Puma
d = np.array([0.67183, 0.13970, 0, 0.43180, 0, 0.0565])
alpha = np.array([-np.pi/2, 0, np.pi/2, -np.pi/2, np.pi/2, 0])
a = np.array([0,0.43180, -0.02032, 0,0,0])
bot = DhRobot(alpha, a, d)
print(bot.hasKnownDecomposition())
print(bot.fwdKin(np.array([1,1,1,1,1,1])))
