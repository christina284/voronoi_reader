#!/usr/bin/env python
import rospy
from tuw_multi_robot_msgs.msg import Graph, Vertex
import numpy as np
from geometry_msgs.msg import Point
from itertools import combinations
from voronoi_reader.msg import Vertx, Edges
import csv
import copy
from rospy_message_converter import message_converter
import pandas as pd
from alphashape import alphashape
import matplotlib.pyplot as plt
from descartes import PolygonPatch
from itertools import combinations
from geometry_msgs.msg import PolygonStamped, Polygon, PointStamped
from visualization_msgs.msg import MarkerArray


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

    # convert back to Graph msg
    def convert_back(vor):
        v_msg = []
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

    VertxArray_unq = []
    positions = []
    # delete multiple vertices with same position
    for i in VertxArray:
        if i.pos not in positions:
            VertxArray_unq.append(i)
            positions.append(i.pos)

    EdgesArray = []
    max_len = max([x.node_id for x in VertxArray_unq])
    it = 0
    for i in range(len(vd)):
        # we iterate at vd because we want all the edges
        e = Edge(edge_id=i, source=0, target=-1, line=vd[i].path, length=0, width=vd[i].width)

        # find start of nodes
        e.source = [x.node_id for x in VertxArray_unq if x.pos == vd[i].path[0]][0]

        # find node_id that corresponds to the end of the edge
        e.target = [x.node_id for x in VertxArray_unq if x.pos == vd[i].path[-1]]

        # if at the end there is no vertex, create a vertex
        if not e.target:
            it += 1
            v = Vertx(node_id = max_len + it, pos=e.line[-1], degree=0, width=1, edges=[])
            e.target = v.node_id
            VertxArray_unq.append(v)
            del v
        else:
            e.target = e.target[0]

        # find length of edge
        e.length = comp_dist(vd[i].path)

        EdgesArray.append(e)
        del e


    # find degree - edges that start or end in a vertex
    for i in VertxArray_unq:
        i.edges = [x.edge_id for x in EdgesArray if x.line[0] == i.pos or x.line[-1] == i.pos]
        i.degree = len(i.edges)
         


    # algorithm 1:-Filter with obstacle distance
    td = 0.3
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
            # pt_msg = PointStamped()
            # pt_msg.header = msg.header
            # pos_temp= copy.copy(i.pos)
            # pos_temp.x += msg.origin.position.x 
            # pos_temp.y += msg.origin.position.y
            # pt_msg.point = pos_temp
            # pub5 = rospy.Publisher('points_with_deg_not_2_', PointStamped, queue_size=1)
            # pub5.publish(pt_msg)
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
        E = []
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
            edges_joined = []
            while True:
                l_sum += e_curr.length
                ids_gone.append(e_curr.edge_id)
                e_new.line.extend(e_curr.line)

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

    
    e2 = skipping_edges(VD1['Vertices'], VD1['Edges'], v0)
    for a, b in combinations(e2, 2):
        if a.source == b.source:
            print(a.source, b.source, a.edge_id, b.edge_id, [x.edges for x in v0 if x.node_id == a.source])
        elif a.source == b.target:
            print(a.source, b.target,a.edge_id, b.edge_id, [x.edges for x in v0 if x.node_id == a.source])

        elif a.target == b.target:
            print(a.target, b.target, a.edge_id, b.edge_id,[x.edges for x in v0 if x.node_id == a.target])

        elif a.target == b.source:
            print(a.target, b.source,a.edge_id, b.edge_id, [x.edges for x in v0 if x.node_id == a.target])

    VD2_1 = {'Vertices': v0, 'Edges': e2}   # graph after skip edges and skip vertices

    v1_msg = convert_back(VD2_1)
    new_msg = msg
    new_msg.vertices = v1_msg
    pub = rospy.Publisher('new_graph', Graph, queue_size=1)
    pub.publish(new_msg)

    VD2 = VD2_1.copy()
    # def intermediates(p1, p2, nb_points=8):
    #     """"Return a list of nb_points equally spaced points
    #     between p1 and p2"""
    #     # If we have 8 intermediate points, we have 8+1=9 spaces
    #     # between p1 and p2
    #     x_spacing = (p2.x - p1.x) / (nb_points + 1)
    #     y_spacing = (p2.y - p1.y) / (nb_points + 1)
    #     p = []
    #     for i in range(1, nb_points + 1):
    #         p.append(Point(p1.x + i * x_spacing, p1.y + i * y_spacing, 0))
    #     return p
    def intermediates(p1, p2, parts):
        p = zip(np.linspace(p1.x, p2.x, parts+1),
                    np.linspace(p1.y, p2.y, parts+1))
        path = [Point(x,y,0) for (x,y) in p]
        return path

    def vertex_merging(ver, edges, sigma):
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

    v3, e3 = vertex_merging(VD2['Vertices'], VD2['Edges'], 1)
    VD3 = {'Vertices': v3, 'Edges': e3}   # graph after merging vertices

    # alphashape
    points, points2 = [], []
    for i in v0:
        point = tuple((i.pos.x, i.pos.y))
        points.append(point)
     
    # Define alpha parameter
    alpha = 0.1

    # # Generate the alpha shape
    alpha_shape = alphashape(points, alpha)

    for xc,yc in zip(*alpha_shape.exterior.coords.xy):
        p = Point(x=xc + msg.origin.position.x, y=yc + msg.origin.position.y, z=0.0)
        points2.append(p)
    Poli = Polygon(points2)

    # # Initialize plot
    # fig, ax = plt.subplots()
    # # Plot input points
    # ax.scatter(*zip(*points))

    # # Plot alpha shape
    # ax.add_patch(PolygonPatch(alpha_shape, alpha=.2))

    # plt.show()
    polstmp = PolygonStamped(header=msg.header, polygon=Poli)
    pub2 = rospy.Publisher('alpha_shape', PolygonStamped, queue_size=1)
    pub2.publish(polstmp)




    print('done, lets go convert back')


    # Test of only two points to see visualization
    # pp = []
    # nzptz  = []
    # for i in range(1,3):
    #     pl = Vertex(i, True, [], 0, 0, [], [])
    #     # 'id', 'valid', 'path', 'weight', 'width', 'successors', 'predecessors']
    #     # pl.source = i
    #     pl.path = [Point(x = (i-1)**2+ 1.0 ,y = (i-1)*2 + 0.0, z=0.0), Point(x = i**2 + 1.0 ,y = i*2 + 0.0, z=0.0)]
    #     print('pl path', pl.path)
    #     pp.extend(pl.path)
    #     print('pp', pp)
    #     nzptz.append(pl)
    #     del pl
    # # pp = [p[0] for p in pp]
    # vvv = [Vertex(1, True, pp, 0, 0, [] ,[])]
    # new2_msg = msg
    # new2_msg.vertices = nzptz

    # pub4 = rospy.Publisher('graph_after_prunning', Graph, queue_size=1)
    # pub4.publish(new2_msg)
    # new3_msg = msg
    # new3_msg.vertices = vvv
    # pub3 = rospy.Publisher('graph_after_prunning1', Graph, queue_size=1)
    # pub3.publish(new3_msg)

    v2_msg = convert_back(VD3)
    new2_msg = msg
    new2_msg.vertices = v2_msg
    pub4 = rospy.Publisher('graph_after_prunning', Graph, queue_size=1)
    pub4.publish(new2_msg)

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
