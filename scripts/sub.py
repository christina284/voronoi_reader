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

    def __init__(self, edge_id=0, source=0, target=-1, line=[], length=0, width=1):
        self.edge_id = edge_id
        self.source = source
        self.target = target
        self.line = line
        self.length = length
        self.width = width


def callback(msg):

    vd = msg.vertices
    # create list of nodes
     # create list of nodes
    VertxArray = []
    for i in range(len(vd)):
        v = Vertx(node_id=i, pos=vd[i].path[0], degree=0, width=vd[i].width, edges=[])
        VertxArray.append(v)
        del v

    VertxArray_unq = []
    positions = []
    # delete multiple vertices with same position
    for i in VertxArray:
        if i.pos not in positions:
            VertxArray_unq.append(i)
            positions.append(i.pos)

    EdgesArray = []
    max_len = len(VertxArray_unq)
    k = 0
    for i in range(len(vd)):
        # we iterate at vd because we want all the edges
        e = Edge(edge_id=i, source=0, target=-1, line=vd[i].path, length=0, width=vd[i].width)

        # find start of nodes
        e.source = [x.node_id for x in VertxArray_unq if x.pos == vd[i].path[0]][0]

        # find node_id that corresponds to the end of the edge
        e.target = [x.node_id for x in VertxArray_unq if x.pos == vd[i].path[-1]]

        # if at the end there is no vertex, create a vertex
        if not e.target:
            k += 1
            v = Vertx(node_id=max_len + k, pos=e.line[-1], degree=1, width=1, edges=[])
            e.target = v.node_id
            VertxArray_unq.append(v)
            del v
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


    # algorithm 1:-Filter with obstacle distance
    td = 0.5
    e1 = [x for x in EdgesArray if x.width > td]

    # find degree - edges that start or end in a vertex
    for i in VertxArray_unq:
        i.edges = [x.edge_id for x in e1 if x.line[0] == i.pos or x.line[-1] == i.pos]
        i.degree = len(i.edges)

    VD0 = {'Vertices': VertxArray_unq, 'Edges': EdgesArray}
    VD1 = {'Vertices': VertxArray_unq, 'Edges': e1}

    # algorithm 2: Skipping vertices
    v0 = []  # vertices to keep
    for i in VertxArray_unq:
        if i.degree == 1 or i.degree > 2:
            v0.append(i)
    # vertices to skip
    v_skipped = [x for x in VertxArray_unq if x not in v0]  # V^VD1\v0

    def skipping_edges(ver, edges, v0):

        """
        :param ver: all vertices
        :param edges: all edges
        :param v0: nodes with degree != 2
        :return: E: new edges
        """

        ids_gone = []
        E, already_visited = [], []
        for i in range(len(edges)):
            e_curr = edges[i]
            if e_curr.edge_id in ids_gone:
                continue
            p1 = e_curr.source
            # if e_curr starts from a deleted node, skip
            while True:
                # find if e_curr starts from a node with deg != 2
                src = [x for x in v0 if x.node_id == p1]
                # if it doesnt, it means its src has 2 edges, we go to the other one
                if not src:
                    p1_node = [x for x in ver if x.node_id == p1][0]
                    # go to the next edge
                    e_curr = [x for x in edges if x.edge_id in p1_node.edges and x != e_curr][0]
                    # [value_false, value_true][<test>]
                    # if the source of the edges is the same, go to the other target node
                    p1 = [e_curr.source, e_curr.target][p1 == e_curr.source]
                else:
                    break
            v_curr = [x for x in ver if x.node_id == p1][0]  # p1_node
            l_sum = 0
            e_new = Edge(edge_id=e_curr.edge_id, source=v_curr.node_id, target=-1, line=[], length=0)
            while True:
                l_sum += e_curr.length
                ids_gone.append(e_curr.edge_id)
                e_new.line.extend(e_curr.line)
                # find next vertex that is on the edge and it not the current
                # it searches in ver because some edges might end in vertices that are skipped
                v_next = [x for x in ver if e_curr.edge_id in x.edges and x != v_curr][0]
                if v_next.degree != 2:
                    break
                # find the other edge of v_next
                e_next = [x for x in edges if x.edge_id in v_next.edges and x != e_curr][0]
                e_curr = e_next
                v_curr = v_next
            e_new.line.extend(e_curr.line)
            e_new.length = l_sum
            E.append(e_new)
            del e_new
        return E

    e2 = skipping_edges(VD1['Vertices'], VD1['Edges'], v0)
    VD2 = {'Vertices': v0, 'Edges': e2}
    # skipping_edges(VD1['Vertices'], VD1['Edges'], v0)
    print('done, lets go convert back')
    # convert back to Graph msg
    v_msg = []
    vor = VD2
    for i in range(len(vor['Edges'])):
        # msg : id, valid, path, suc, pred, width, weight
        # vtx:  node_id=np.nan, pos=(0, 0), degree=0, width=0, edges=[])
        # edge :edge_id=0, source=0, target=-1, line=[], length=0, ray='False'):
        v = Vertex(i, True, [], [], [], 0, 0)
        s_node = [x for x in vor['Vertices'] if x.node_id == vor['Edges'][i].source][0]

        v.successors = s_node.edges

        v.path = vor['Edges'][i].line
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
