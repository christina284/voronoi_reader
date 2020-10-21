#!/usr/bin/env python
import rospy
from tuw_multi_robot_msgs.msg import Graph, Vertex
import numpy as np
from geometry_msgs.msg import Point
from itertools import combinations
from voronoi_reader.msg import Vertx, Edges
import csv
from rospy_message_converter import message_converter
import pandas as pd
from itertools import chain

class Vertx:
    def __init__(self, node_id=np.nan, pos=(0, 0), degree=0, width=0, edges=[]):
        self.node_id = node_id
        self.pos = pos
        self.degree = degree
        self.width = width
        self.edges = edges


class Edge:

    def __init__(self, edge_id=0, source=0, target=-1, line=[], length=0, ray='False'):
        self.edge_id = edge_id
        self.source = source
        self.target = target
        self.line = line
        self.length = length
        self.ray = ray


def callback(msg):

    vd = msg.vertices
    # create list of nodes
    VertxArray = []
    for i in range(len(vd)):
        v = Vertx(node_id=i, pos=vd[i].path[0], degree=0, width=vd[i].width, edges=[])
        VertxArray.append(v)
        del v

    VertxArray_unq = []
    positions = []
    # delete multiple vertices with same position
    for i in range(len(VertxArray)):
        if VertxArray[i].pos not in positions:
            VertxArray_unq.append(VertxArray[i])
            positions.append(VertxArray[i].pos)

    EdgesArray = []
    for i in range(len(vd)):  
        # we iterate at vd because we want all the edges
        e = Edge(edge_id=i, source=0, target=-1, line=vd[i].path, length=0, ray=False)

        # find start of nodes
        e.source = [x.node_id for x in VertxArray_unq if x.pos == vd[i].path[0]][0]

        # find node_id that corresponds to the end of the edge
        e.target = [x.node_id for x in VertxArray_unq if x.pos == vd[i].path[-1]]

        # if at the end there is no vertex, it is a ray
        if e.target == []:
            e.target = -1
            e.ray = True
        else:
            e.target = e.target[0]
        
        # find length of edge
        xcoords, ycoords = [], []
        for j in range(len(vd[i].path)):
            xcoords.append(vd[i].path[j].x)
            ycoords.append(vd[i].path[j].y)
        sum_dist = 0
        for k in range(len(xcoords) - 1):
            sum_dist += np.sqrt((xcoords[k + 1] - xcoords[k])
                                ** 2 + (ycoords[k + 1] - ycoords[k]) ** 2)
        e.length = sum_dist
        EdgesArray.append(e)
        del e

    # find degree - edges that start or end in a vertex
    for i in VertxArray_unq:
        i.edges = [x.edge_id for x in EdgesArray if x.line[0] == i.pos or x.line[-1] == i.pos]
        i.degree = len(i.edges)
        

    # eliminate vertices that are the ends of two rays
    rays = [x for x in EdgesArray if x.ray]

    for i, j in combinations(range(len(rays)), 2):
        # if two rays are juncted, there is no node
        if i != j:
            point = EdgesArray[i].line[-1]
            p = [x for x in EdgesArray if x.line[-1]== point or x.line[0]== point]
            if point == EdgesArray[j].line[-1] and not p:
                # we add its path to the second, reversed
                edge_ = EdgesArray[i]
                s = edge_.source
                vtx_ = [x for x in VertxArray_unq if x.node_id == s][0]
                EdgesArray[j].line += reversed(edge_.line[:-1])
                # we go to deleted edge's source, and delete it as edge
                vtx_.edges.remove(edge_.edge_id)
                if EdgesArray[j].edge_id not in vtx_.edges:
                    vtx_.edges.append(EdgesArray[j].edge_id)
                else:
                    vtx_.degree -= 1
                # we delete the first edge
                edge_.edge_id = -10

    EdgesArray = [x for x in EdgesArray if x.edge_id != -10]


    # algorithm 1:-Filter with obstacle distance
    td = 0.01
    VertxArray_unq = [x for x in VertxArray_unq if x.width > td]

    # algorithm 2: Skipping vertices
    vd1, vd2 = [], []
    for i in VertxArray_unq:
        if i.degree == 0:
            vd1.append(i)
            # continue
        if i.degree == 2:
            # if none of the edges are rays, skip the vertex
            if (not EdgesArray[i.edges[0]].ray) and (not EdgesArray[i.edges[1]].ray):
                vd1.append(i)
                # continue
        vd2.append(i)

    E  = []
    for i in range(len(VertxArray_unq)):
        e_curr = EdgesArray[VertxArray_unq[i].edges[0]]
        v_curr = VertxArray_unq[i]
        l_sum = 0
        e_new = Edge(edge_id=i, source=VertxArray_unq[i].node_id, target=-1, line=[], length=0, ray=False)
        while True:
            l_sum += e_curr.length
            e_new.line.append(v_curr.pos)

            # go to the node in the end of the edge
            v_next = [x for x in VertxArray_unq if x.node_id == e_curr.target][0]
            e_new.target = v_next.node_id
            next_pos = v_next.pos

            if v_next.degree != 2:
                print('Broke because degree diff from 2', i)
                break

            # the vertex has only two edges - go to the other edge
            i_next = [j for j in v_next.edges if j != VertxArray_unq[i].node_id][0]

            e_next = [e for e in EdgesArray if e.edge_id == i_next][0]
            if e_next.ray:
                print('Broke because of ray', i)
                break

            e_curr = e_next
            v_curr = v_next
        e_new.line.append(next_pos)
        e_new.length = l_sum
        E.append(e_new)
        del e_new

   
    E = EdgesArray

    v_msg = []
    for i in range(len(E)):
        # msg : id, valid, path, suc, pred, width, weight
        # vtx:  node_id=np.nan, pos=(0, 0), degree=0, width=0, edges=[])
        # edge :edge_id=0, source=0, target=-1, line=[], length=0, ray='False'):
        v = Vertex(i, True, [], [], [], 0,0)
        s_node = [x for x in VertxArray_unq if x.node_id == E[i].source][0]
        if not E[i].ray:
            t_node = [x for x in VertxArray_unq if x.node_id == E[i].target][0]
            v.successors = [s_node.edges, t_node.edges]
        else:
            v.successors = s_node.edges

        v.path = E[i].line
        v.width = s_node.width
        v.weight = 1
        v.predecessors = []
        v.successors = []
        v_msg.append(v)
        del v
    
    new_msg = msg
    # new_msg.vertices = [v_msg[6], v_msg[10]]
    new_msg.vertices = v_msg
    pub = rospy.Publisher('new_graph', Graph, queue_size=1)
    pub.publish(new_msg)

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
