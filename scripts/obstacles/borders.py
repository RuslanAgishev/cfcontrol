import numpy as np
import matplotlib.pyplot as plt
from math import *
from scipy.spatial import ConvexHull


def circles(point, R):
	t = np.linspace(0,1,30)
	x = point[0] + R*np.cos(2*pi*t)
	y = point[1] + R*np.sin(2*pi*t)
	C = np.array([x,y]).T
	return C

def tangent(point1, point2, R):
	x1 = point1[0]; y1 = point1[1]
	x2 = point2[0]; y2 = point2[1]
	l = np.array([x1-x2, y1-y2])
	L1 = np.zeros((10,2))
	L2 = np.zeros((10,2))
	if np.linalg.norm(l) != 0:
		xc = x1 + (y2-y1) * R / np.linalg.norm(l)
		yc = y1 + (x1-x2) * R / np.linalg.norm(l)
		xd = x2 + (y2-y1) * R / np.linalg.norm(l)
		yd = y2 + (x1-x2) * R / np.linalg.norm(l)
		L1[:,0] = np.linspace(xc,xd, 10)
		L1[:,1] = np.linspace(yc,yd, 10)
		xe = x1 - (y2-y1) * R / np.linalg.norm(l)
		ye = y1 - (x1-x2) * R / np.linalg.norm(l)
		xf = x2 - (y2-y1) * R / np.linalg.norm(l)
		yf = y2 - (x1-x2) * R / np.linalg.norm(l)
		L2[:,0] = np.linspace(xe,xf, 10)
		L2[:,1] = np.linspace(ye,yf, 10)
	return L1,L2


def intersection(lst1, lst2): 
    lst3 = [value for value in lst1 if value in lst2] 
    return (lst3 != [])

def close_pts(points, R):
	M = []; C = []; L1 = []; L2 = []
	G_global = np.array([0,0])
	for i in range(len(points)):
		M.append([])
	for i in range(len(points)):
		C.append( circles(points[i],R) )
		G_global = np.vstack((G_global,np.array(C[-1])))
		for j in range(len(points)):
			if np.linalg.norm(points[i]-points[j]) <= 2*R:
				M[i].append(j)
				l1,l2 = tangent(points[i], points[j], R)
				L1.append(l1); L2.append(l2)
				if j>i:
					G_global = np.vstack((G_global,L1[-1]))
					G_global = np.vstack((G_global,L2[-1]))
	G_global = G_global[1:]

	groups = []
	for i in range(len(points)):
		for j in range(i,len(points)):
			if intersection(M[i],M[j]):
				group = M[i] + list(set(M[j])-set(M[i]))
				groups.append( group )

	for i in range(len(groups)):
		for j in range(len(groups)):
			if intersection(groups[i],groups[j]) and j!=i:
				if len(groups[i])<len(groups[j]):
					groups[i] = []
				else:
					groups[j] = []
	group_numbers = []
	for g in groups:
		if g != []:
			group_numbers.append(g)
	

	G_parts = []
	for i in range(len(group_numbers)):
		G_parts.append(np.array([0,0]))
	for i in range(len(group_numbers)):
		for g in ( (group_numbers[i]) ):
			G_parts[i] = np.vstack((G_parts[i],C[g]))
		G_parts[i] = G_parts[i][1:]

	return group_numbers, G_global, G_parts

# INITIALIZATION
legend_list = []
N = 9
R = 0.15
points = np.random.rand(N, 2)
group_numbers, G_global, G_parts =  close_pts(points, R)
print "Obstacles' numbers in groups:\n"+str(group_numbers)

# VISUALIZATION
plt.figure(figsize=(20,8))
ax1 = plt.subplot(121)
ax1.set_aspect('equal')
for g in range(len(G_parts)):
	G = G_parts[g]
	legend_list.append("group"+str(g))
	for i in range(len(points)):
		plt.plot(G[:,0], G[:,1], 'o')
	hull = ConvexHull(G)
	for simplex in hull.simplices:
		plt.plot(G[simplex, 0], G[simplex, 1], 'k-')
plt.title('Obstacles divided into groups')
plt.plot(points[:,0], points[:,1], 'x')
plt.legend(legend_list)

plt.subplot(122)
ax2 = plt.subplot(122)
ax2.set_aspect('equal')
for i in range(len(points)):
	plt.plot(G_global[:,0], G_global[:,1], 'o')
	plt.title('All obstacles'' borders')
plt.show()

