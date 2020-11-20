#!/usr/bin/env python3

import numpy as np
import pickle
import copy
import rospy
from tuw_multi_robot_msgs.msg import Graph, Vertex
from geometry_msgs.msg import Point


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



 # algorithm 2: Skipping vertices
def skip_vertices(arr):
    arr2 = []  # vertices to keep
    for i in arr:
        if i.degree == 1 or i.degree > 2:
            arr2.append(i)
    return arr2



def skipping_edges(ver, edges, v0):

    """
    :param ver: all vertices
    :param edges: all edges
    :param v0: nodes with degree != 2
    :return: E: new edges
    """
    print('skipping edges')
    pickle.dump(ver, open("/home/chris/Desktop/ver.pkl", "wb"))
    pickle.dump(v0, open("/home/chris/Desktop/v0.pkl", "wb"))
    pickle.dump(edges, open("/home/chris/Desktop/edges.pkl", "wb"))

    ids_gone = []
    E = []
    for i in range(len(edges)):
        e_curr = edges[i]
        if e_curr.edge_id in ids_gone:
            continue
        pt = []

        p1 = copy.copy(e_curr.source)
        # if e_curr starts from a deleted node, skip
        while True:
            # find if e_curr starts from a node with deg != 2
            src = [x for x in v0 if x.node_id == p1]
            
            # if it doesnt, it means its src has 2 edges, we go to the other one
            if not src:
                p1_node = [x for x in ver if x.node_id == p1][0]
                pt = p1_node.node_id
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
        edges_joined = []
        while True:
            l_sum += e_curr.length
            e_new.line.extend(e_curr.line)
            ids_gone.append(e_curr.edge_id)
            # find next vertex that is on the edge and it not the current
            # it searches in ver because some edges might end in vertices that are skipped
            # v_next = [x for x in ver if e_curr.edge_id in x.edges and x != v_curr][0]
            edge_ends = [e_curr.source, e_curr.target]
            v_next = [x for x in ver if x.node_id in edge_ends and x.node_id != v_curr.node_id][0]
            e_new.target = v_next.node_id
            e_new.line.extend([v_next.pos])
            edges_joined.append(e_curr.edge_id)
            if v_next.degree != 2:
                break
            # find the other edge of v_next
            e_next = [x for x in edges if x.edge_id in v_next.edges and x != e_curr][0]
            e_curr = e_next
            v_curr = v_next
        # e_new.line.extend(e_curr.line)
        # change the edges id on vertices, to curr edge id
        for vtx in v0:
            # [x + 1 if x >= 45 else x + 5 for x in l]
            vtx.edges = list(set([e_new.edge_id if x in edges_joined else x for x in vtx.edges]))
            vtx.degree = len(vtx.edges)
        e_new.length = l_sum    
        E.append(e_new)
        del e_new
    return E

def intermediates(p1, p2, parts):
    p = zip(np.linspace(p1.x, p2.x, parts+1),
                np.linspace(p1.y, p2.y, parts+1))
    path = [Point(x,y,0) for (x,y) in p]
    return path


def vertex_merging(ver, edges, sigma):
    print('merging vertices..')
    for e in edges:
        if e.length < sigma:
            # merge source and target
            n1 = [x for x in ver if x.node_id == e.source][0]
            n2 = [x for x in ver if x.node_id == e.target][0]
            pos_of_scs = n1.pos
            pos_of_tgt = n2.pos
            # find their middlepoint
            midpoint = Point((pos_of_scs.x+pos_of_tgt.x)/2, (pos_of_scs.y+pos_of_tgt.y)/2, 0.0)

            # edges of nodes to be merged
            edges_ = list(set(n1.edges + n2.edges))

            # find edges of nodes by their ids
            edges_ = [x for x in edges if x.edge_id in edges_ if x != e]

            # find which edges starts or ends in the deleted vertices
            targs = [x for x in edges_ if x.target == n1.node_id or x.target == n2.node_id]
            scs = [x for x in edges_ if x.source == n1.node_id or x.source == n2.node_id]
            # let the new node have the node_id of n1 (source of edge w/ len<sigma)
            # we change targets and sources of edges of the nodes to be merged
            for k in targs:
                k.target = n1.node_id
            for o in scs:
                o.source = n1.node_id

            # delete nodes and edge 
            ver = [x for x in ver if x not in [n1, n2]]
            edges = [x for x in edges if x != e]
            # create new node
            print('ver',len(ver), 'edg',len(edges))
            new_node = Vertx(node_id=n1.node_id, edges=[x.edge_id for x in edges_],
                                degree=len(edges_), pos=midpoint, width=0)
            ver.extend([new_node])
            # we recreate the edges, with new lengths and lines from the new_node
            for k in (targs + scs):
                t = [x for x in ver if x.node_id == k.target][0]
                s = [x for x in ver if x.node_id == k.source][0]
                k.line = intermediates(s.pos, t.pos, 10)
                k.length = comp_dist(k.line)
    
    return ver, edges


def short_deadend_removal(v, dead_end_max_length):
    edges = v['Edges']
    vertices = v['Vertices']
    # short dead end removal
    for i in edges:
        scs = [x for x in vertices if x.node_id == i.source][0]
        tgt = [x for x in vertices if x.node_id == i.target][0]
        dead_end = [x for x in [scs, tgt] if x.degree == 1]
        if dead_end == []:
            continue
        else:
            dead_end = dead_end[0]
        if dead_end and i.length < dead_end_max_length:
            i.edge_id = 10000
            dead_end.node_id = 10000

    edges_ = [x for x in edges if x.edge_id != 10000]
    vertices_ = [x for x in vertices if x.node_id != 10000]
    return edges_, vertices_

# convert back to Graph msg
def convert_back(vor):
    print('Got in conversion')
    v_msg = []
    for i in range(len(vor['Edges'])):
        # msg : id, valid, path, suc, pred, width, weight
        # vtx:  node_id=np.nan, pos=(0, 0), degree=0, width=0, edges=[])
        # edge :edge_id=0, source=0, target=-1, line=[], length=0, ray='False'):
        # if i == 5 :
        #     print('id 5 ', vor['Edges'][i].__dict__)
        
        v = Vertex(i, True, [], [], [], 0, 0)

        try:
            s_node = [x for x in vor['Vertices'] if x.node_id == vor['Edges'][i].source][0]
        except IndexError:
            print("error")
            # print( [x.node_id for x in vor['Vertices']], 'edz', vor['Edges'][i].__dict__ )

        v.successors = s_node.edges

        v.path = vor['Edges'][i].line
        v.width = s_node.width
        v.weight = 1
        v.predecessors = []
        v_msg.append(v)
        # if i == 7 :
        #     print('id 7 ', vor['Edges'][i].__dict__, v)
        del v
    return v_msg


def comp_dist(path):
    xcoords, ycoords = [], []
    for j in range(len(path)):
        xcoords.append(path[j].x)
        ycoords.append(path[j].y)
    sum_dist = 0
    for k in range(len(xcoords) - 1):
        sum_dist += np.sqrt((xcoords[k + 1] - xcoords[k])
                            ** 2 + (ycoords[k + 1] - ycoords[k]) ** 2)
    return sum_dist