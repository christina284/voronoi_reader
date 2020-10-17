#!/usr/bin/env python
import rospy
from tuw_multi_robot_msgs.msg import Graph
import numpy as np
from geometry_msgs.msg import Point



class Vertx:
	def __init__(self, node_id = np.nan, pos = (0,0), degree = 0, width = 0):
		self.node_id = node_id
		self.pos = pos
		self.degree = degree
		self.width = width

class Edge:
	def __init__(self, edge_id = 0, source = 0, target = 0, line = [], length = 0):
		self.edge_id = edge_id
		self.source =source
		self.target = target
		self.line = line
		self.length = length
		
def callback(msg):

	VertxArray = []

	for i in range(len(vd)):
		v = Vertx()
		v.node_id = i
		v.pos = vd[i].path[0] #(vd[i].path[0].x, vd[i].path[0].y)   # beginning point is our position
		neigh = [vd[i].predecessors + vd[i].successors]
		neigh =  [x for t in neigh for x in t]
		for j in range(len(neigh)):
			if (vd[j].path[0] == vd[i].path[0]) or (vd[j].path[-1] == vd[i].path[0]):
				# if neighbor ends or starts at v.pos, then it is an edge
				v.degree += 1
		v.width = vd[i].width
		VertxArray.append(v)
		del v

	# print(VertxArray[0].__dict__)
	pos_arr = [VertxArray[x].pos for x in range(len(VertxArray))]
	pos_arr = set(pos_arr)
	VertxArray_unq = []
	positions = []

	#delete multiple vertices with same position
	for i in range(len(VertxArray)):
		if VertxArray[i].pos not in positions:
			VertxArray_unq.append(VertxArray[i])
			positions.append(VertxArray[i].pos)
		
	EdgesArray = []
	for i in range(len(vd)):
		e = Edge()
		e.edge_id = i
		e.source = i
		for j in range(len(VertxArray_unq)):
			# print(VertxArray_unq[j].pos, vd[i].path[-1])			
			if VertxArray_unq[j].pos == vd[i].path[-1]:
				e.target = VertxArray_unq[j].node_id
				break
		e.line = vd[i].path
		xcoords, ycoords = [], []
		for j in range(len(vd[i].path)):
			xcoords.append(vd[i].path[j].x)
			ycoords.append(vd[i].path[j].y)
		sum_dist = 0
		for k in range(len(xcoords) - 1):
			sum_dist += np.sqrt((xcoords[k+1] - xcoords[k])**2 + (ycoords[k+1] - ycoords[k])**2)
		e.length = sum_dist
		EdgesArray.append(e)
		del e

	# all vertices with a degree of zero or two are filtered out
    # algorithm 1 -Filter with obstacle distance

	# td = 0.01
	# for i in range(len(vd0)):
	# 	if vd0[i].width <= td:
	# 		vd0[i] = np.nan
			
	# msg.vertices = [i for i in vd0 if i is not np.nan]
	# pub = rospy.Publisher('prunned', Graph, queue_size=1)
	# pub.publish(msg)

	vd = msg.vertices



	return vd


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    rospy.init_node('listener', anonymous=True)
    sub = rospy.Subscriber("segments", Graph, callback)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
