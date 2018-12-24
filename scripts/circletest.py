import numpy as np
from swarmlib import *
import matplotlib.pyplot as plt

obstacle = Obstacle('obstacle0', 0, 0.3)
C = obstacle.circle_points(0.55)
l = np.vstack([np.linspace(0,0.5,100), 0.2*np.ones(100)]).T

from scipy.spatial.distance import cdist
def intersection(curve1, curve2):
	dists = cdist(curve1,curve2)
	i1, i2 = np.where(dists==np.min(dists))
	i1 = i1[0]; i2 = i2[0]
	return np.array([curve1[i1,0], curve1[i1,1]])

X, Y = intersection(C, l)

plt.plot(C[:,0], C[:,1])
plt.plot(l[:,0], l[:,1])
plt.plot(X, Y, 'ro')
plt.show()

