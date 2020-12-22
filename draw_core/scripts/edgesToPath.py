#!/usr/bin/env python
# -*- coding: utf-8 -*
import copy
import modern_robotics
import rospy
import numpy as np
import matplotlib.pyplot as plt
import pylab as pl
from matplotlib import collections  as mc

import cv2
from matplotlib import pyplot as plt


def resize_edge(image_in):
    img = image_in
    h, w = img.shape[:2]
    frame_size = 40
    if h > w:
        h_new = frame_size
        w_new = (frame_size * w) / h
    else:
        w_new = frame_size
        h_new = (frame_size * h) / w
    resizes = cv2.resize(img, (h_new, w_new), cv2.INTER_AREA)

    # Canny edge
    edges = cv2.Canny(resizes, 200, 400)

    traj = []
    r, c = edges.shape
    edges = np.array(edges)
    return edges


def getLinesFromPoints(points):
	n = len(points)
	if (n == 0):
		return []
	elif (n == 1):
		return (points[0], points[0])
	else:
		# print points
		lines = []
		last = None
		newPoint = True
		for p in points:
			# print p
			if newPoint:
				last = p
				newPoint = False
			else:
				if -1 not in p:
					lines.append([last, p])
					last = p
				else:
					newPoint = True

		return lines

def plotResults(results):
	plt.ion()
	plt.axis([0, w_new, 0, h_new])
	for item in results:
		if -1 not in item:
			plt.plot(item[1], h_new - item[0], 'b.')
    		plt.draw()
    		plt.pause(0.0001)

	plt.show()
	# lines = getLinesFromPoints(results[:500])
	#lc = mc.LineCollection(lines, linewidths=2)
	# linesSoFar = []
	# for line in lines:
	# 	linesSoFar.append(line)
	# 	lc = mc.LineCollection(linesSoFar, linewidths=2)
	# 	fig, ax = pl.subplots()
	# 	ax.add_collection(lc)
	# 	ax.autoscale()
	# 	ax.margins(0.1)
	# 	fig.show()
	# 	pl.pause(0.01)
	# 	pl.close(fig)

	# lines = getLinesFromPoints(results)
	# for i in range(0, len(results)):
	# 	((x1,y1),(x2,y2)) = results[i]
	# 	plt.plot([x1,x2], [y1,y2], 'ro-')
	# plt.show()

def isAdjacent(l1, l2):
	(x1, y1) = l1
	(x2, y2) = l2
	return (abs(x2-x1) <= 1) and (abs(y2-y1) <= 1)

def findMostRecentNeighbor(ls, l):
	temporalOrder = ls[::-1]
	for elem in temporalOrder:
		if isAdjacent(elem, l):
			return elem
	return l

def separateBranches(ls):
	last = ls[0]
	res = [last]
	# print ls
	for l in ls[1:]:
		if not isAdjacent(l, last):
			# print l, last
			res.append((-1,-1))
			res.append(findMostRecentNeighbor(res, l))
			res.append(l)
			last = l
		else:
			last = l
			res.append(l)
	return res

def dfs(mapvals, checked, i, j):
	# right and down, no up and left
	#print "dfs ", i, j
	resTmp = []
	checkedTmp = copy.deepcopy(checked)
	if (i-1 >= 0) and (mapvals[i-1][j] > 0) and ((i-1,j) not in checkedTmp):
		#if resTmp != []:
			#print (i,j), (i-1,j)
		resTmp.append((i-1,j))
		checkedTmp.add((i-1,j))
		(innerRes, innerChecked) = dfs(mapvals, checkedTmp, i-1, j)
		checkedTmp = checkedTmp.union(innerChecked)
		if (innerRes != [] and innerChecked != []):
			#resTmp += [(i,j)] + innerRes
			resTmp += innerRes
	if ((i-1 >= 0) and (j-1 >= 0)) and (mapvals[i-1][j-1] > 0) and ((i-1,j-1) not in checkedTmp):
		#if resTmp != []:
			#print (i,j), (i-1,j-1)
		resTmp.append((i-1,j-1))
		checkedTmp.add((i-1,j-1))
		(innerRes, innerChecked) = dfs(mapvals, checkedTmp, i-1, j-1)
		checkedTmp = checkedTmp.union(innerChecked)
		if (innerRes != [] and innerChecked != []):
			#resTmp += [(i,j)] + innerRes
			resTmp += innerRes
	if (j-1 >= 0) and (mapvals[i][j-1] > 0) and ((i, j-1) not in checkedTmp):
		#if resTmp != []:
			#print (i,j), (i,j-1)
		resTmp.append((i,j-1))
		checkedTmp.add((i,j-1))
		(innerRes, innerChecked) = dfs(mapvals, checkedTmp, i, j-1)
		checkedTmp = checkedTmp.union(innerChecked)
		if (innerRes != [] and innerChecked != []):
			#resTmp += [(i,j)] + innerRes
			resTmp += innerRes
	if (i+1 < len(mapvals)) and (mapvals[i+1][j] > 0) and ((i+1,j) not in checkedTmp):
		#if resTmp != []:
			#print (i,j), (i+1,j)
		resTmp.append((i+1,j))
		checkedTmp.add((i+1,j))
		(innerRes, innerChecked) = dfs(mapvals, checkedTmp, i+1, j)
		checkedTmp = checkedTmp.union(innerChecked)
		if (innerRes != [] and innerChecked != []):
			#resTmp += [(i,j)] + innerRes
			resTmp += innerRes
	if ((i+1 < len(mapvals)) and (j+1 < len(mapvals[0]))) and (mapvals[i+1][j+1] > 0) and ((i+1,j+1) not in checkedTmp):
		#if resTmp != []:
			#print (i,j), (i+1,j+1)
		resTmp.append((i+1,j+1))
		checkedTmp.add((i+1,j+1))
		(innerRes, innerChecked) = dfs(mapvals, checkedTmp, i+1, j+1)
		checkedTmp = checkedTmp.union(innerChecked)
		if (innerRes != [] and innerChecked != []):
			#resTmp += [(i,j)] + innerRes
			resTmp += innerRes
	if (j+1 < len(mapvals[0])) and (mapvals[i][j+1] > 0) and ((i, j+1) not in checkedTmp):
		#if resTmp != []:
			#print (i,j), (i,j+1)
		resTmp.append((i,j+1))
		checkedTmp.add((i,j+1))
		(innerRes, innerChecked) = dfs(mapvals, checkedTmp, i, j+1)
		checkedTmp = checkedTmp.union(innerChecked)
		if (innerRes != [] and innerChecked != []):
			#resTmp += [(i,j)] + innerRes
			resTmp += innerRes
	if ((i-1 >= 0) and (j+1 < len(mapvals[0]))) and (mapvals[i-1][j+1] > 0) and ((i-1,j+1) not in checkedTmp):
		#if resTmp != []:
			#print (i,j), (i-1,j+1)
		resTmp.append((i-1,j+1))
		checkedTmp.add((i-1,j+1))
		(innerRes, innerChecked) = dfs(mapvals, checkedTmp, i-1, j+1)
		checkedTmp = checkedTmp.union(innerChecked)
		if (innerRes != [] and innerChecked != []):
			#resTmp += [(i,j)] + innerRes
			resTmp += innerRes
	if ((i+1 < len(mapvals)) and (j-1 >= 0)) and (mapvals[i+1][j-1] > 0) and ((i+1,j-1) not in checkedTmp):
		#if resTmp != []:
			#print (i,j), (i+1,j-1)
		resTmp.append((i+1,j-1))
		checkedTmp.add((i+1,j-1))
		(innerRes, innerChecked) = dfs(mapvals, checkedTmp, i+1, j-1)
		checkedTmp = checkedTmp.union(innerChecked)
		if (innerRes != [] and innerChecked != []):
			#resTmp += [(i,j)] + innerRes
			resTmp += innerRes
	return (resTmp, checkedTmp)

def getRepeatedPoints(pointList):
	last = None
	repeatNext = False
	res = []
	for i in xrange(len(pointList)):
		elem = pointList[i]
		if repeatNext:
			res.append(elem)
			repeatNext = False
			last = elem

		if -1 in elem:
			repeatNext = True
			res.append(last)
		else:
			res.append(elem)
			last = elem
	return res

def getPointsFromEdges(edgemap):
	# This function takes a 2D array of 1's and 0's and returns the a point to point
	# path along edges separated by (-1, -1) which indicates that the end-effector
	# should be lifted in the positive z-direction because the edges are either disconnected
	# or fragmented into a different part
    checked = set()
    res = []
    for i in xrange(len(edgemap)):
        for j in xrange(len(edgemap[i])):
            if ((i,j) not in checked) and (edgemap[i][j] > 0):
                checked.add((i,j))
                #res += [(i,j)]
                (resTmp, checkedTmp) = dfs(edgemap, checked, i, j)
                # add default value to say pick up end-effector
                checked = checked.union(checkedTmp)
                if (resTmp != []):
                    res.append([(i,j)] + resTmp)
                else:
                    res.append([(i,j)])
            else:
                checked.add((i,j))
    res = filter(lambda branch: len(branch) > 1, res)
    return reduce(lambda a,b: a + [(-1,-1)] + b, filter(lambda x: x!=[], map(lambda tree: separateBranches(tree),res)))
	#return filter(lambda x: x!=[], map(lambda tree: separateBranches(tree),res))
